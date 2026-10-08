/**
 * @file    bsp_position_gpio.h
 * @brief   四路位置 GPIO 输入及下降沿事件接口
 * @layer   BSP
 *
 * 引脚映射（低电平有效，进入对应位置时产生下降沿）：
 *   4WD_LOCK 端点  -> PTA0 / GPIOA0
 *   4WD 右位置     -> PTA1 / GPIOA1
 *   4WD 左位置     -> PTD3 / GPIOD3
 *   2WD 端点       -> PTB2 / GPIOB2
 *
 * 物理顺序：
 *   2WD端点 -- 4WD左位置 -- 4WD右位置 -- 4WD_LOCK端点
 *
 * 约束：
 *   1. PINS_DRV_Init() 只在 main() 中执行一次；
 *   2. 本模块不重复配置 MUX、输入方向、上下拉和中断模式；
 *   3. pin_mux 必须把四路配置为 GPIO 输入、下降沿中断；
 *   4. 上电时输入已经为低不会产生下降沿，因此仍需读取当前电平；
 *   5. YTM32B1L GPIO 中断按端口划分：GPIOA/B/D_IRQn；
 *   6. ISR 只锁存下降沿事件并更新端点联锁，不直接操作 PWM，也不运行应用状态机。
 *
 * 数据通路说明：
 *   - “当前电平”由 BSP_PositionGpio_Sample() 周期读取，用于判断输入现在
 *     是否仍然处于低电平有效状态；
 *   - “下降沿事件”由 GPIOA/B/D_IRQHandler() 异步锁存，用于记录两个
 *     控制周期之间曾经出现过的短脉冲；
 *   - TASK 层会把锁存事件在一个控制周期内合并为有效输入。对于 4WD 目标，
 *     PTD3 或 PTA1 的下降沿/低电平会被直接解释为“4WD 到位”。
 *
 * 并发约定：
 *   - 中断上下文只写 s_fallingEventMask，并更新端点方向联锁；
 *   - 主循环任务通过 ConsumeFallingEvents() 原子取走事件；
 *   - 输入快照由 1 ms 周期任务更新，APP 状态机不直接访问 GPIO 寄存器。
 */

#ifndef BSP_POSITION_GPIO_H
#define BSP_POSITION_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include "pins_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PINS_DRV_ReadPin/GetPortIntFlag/ClearPinIntFlagCmd 的端口参数
 * 均使用 GPIO_Type 基址（GPIOA/GPIOB/GPIOD），不使用 PCTRLx。
 */
/* 4WD_LOCK 机械端点：PTA0，向 FORWARD 运动时属于危险端点。 */
#define BSP_POS_LOCK_END_GPIO       GPIOA
#define BSP_POS_LOCK_END_PIN        (0U)

/* 4WD 右位置点：PTA1，从锁止侧反转进入 4WD 时命中即到位。 */
#define BSP_POS_4WD_RIGHT_GPIO      GPIOA
#define BSP_POS_4WD_RIGHT_PIN       (1U)

/* 4WD 左位置点：PTD3，从 2WD 侧正转进入 4WD 时命中即到位。 */
#define BSP_POS_4WD_LEFT_GPIO       GPIOD
#define BSP_POS_4WD_LEFT_PIN        (3U)

/* 2WD 机械端点：PTB2，向 REVERSE 运动时属于危险端点。 */
#define BSP_POS_2WD_END_GPIO        GPIOB
#define BSP_POS_2WD_END_PIN         (2U)

/*
 * 下降沿事件位定义。每一位既可表示“本周期曾出现下降沿”，也可用于
 * BSP_PositionGpio_GetActiveMask() 表示“当前低电平有效”。两种数据的
 * 语义由调用接口区分，但位位置保持一致，便于 TASK 层直接合并。
 */
/* 位序号，供 APP 层按运动方向选择停车霍尔时使用。 */
#define BSP_POS_IDX_LOCK_END        (0U)
#define BSP_POS_IDX_4WD_RIGHT       (1U)
#define BSP_POS_IDX_4WD_LEFT        (2U)
#define BSP_POS_IDX_2WD_END         (3U)

#define BSP_POS_EVENT_NONE          (0x00U)
#define BSP_POS_EVENT_LOCK_END      (1U << BSP_POS_IDX_LOCK_END)
#define BSP_POS_EVENT_4WD_RIGHT     (1U << BSP_POS_IDX_4WD_RIGHT)
#define BSP_POS_EVENT_4WD_LEFT      (1U << BSP_POS_IDX_4WD_LEFT)
#define BSP_POS_EVENT_2WD_END       (1U << BSP_POS_IDX_2WD_END)
#define BSP_POS_EVENT_ALL           (0x0FU)

/**
 * @brief 4WD 停车预置触发。
 *
 * 由 APP 层在出发前武装，由 PTA1（霍尔B）的中断和 1 ms 周期采样共同
 * 判定。命中时直接在中断上下文中制动，避免"1 ms 采样 + 2 ms 控制周期"
 * 的检测延迟转化成停车位置误差。
 *
 * 停车点统一为霍尔B检测区的左边界：
 *   FORWARD 从左侧进入 -> B 由高变低即到左边界 -> ARM_B_FALLING
 *   REVERSE 从右侧进入 -> B 由低变高即出左边界 -> ARM_B_RISING
 *
 * ── 判定完全由 PTA1 的边沿中断完成 ──────────────────────────────
 *
 * PTA1 配置为 PCTRL_INT_EITHER_EDGE。中断只在边沿发生时进入，因此清标志
 * 后回读到的电平直接表明这次是哪个边沿——读到低是下降沿，读到高是上升沿。
 * 这里的回读是给边沿分类，不是轮询电平。
 *
 * 周期采样不参与停车判定。曾经有过一版在 1 ms 采样里按电平重复判定作为
 * 兜底，但周期采样每次都拿到当前电平，REVERSE 武装 ARM_B_RISING 时 B 本
 * 来就是高电平，会立刻误命中；为此加过"先进区再出区"的两阶段门闩，而门
 * 闩又会在进区下降沿丢失时永远卡在第一阶段，使执行器一路冲到机械端点。
 * 去掉那行兜底之后，单阶段判定即可成立，上述问题一并消失。
 *
 * 因此 REVERSE 直接武装 ARM_B_RISING：进区的下降沿进入中断时 bLow 为真，
 * 与 ARM_B_RISING 不匹配而被忽略；出区的上升沿 bLow 为假，匹配并制动。
 * 起步时 B 已经压住的情况同样成立，不需要额外特例。
 */
typedef enum
{
    BSP_POS_ARM_NONE      = 0, /**< 未武装，中断不干预电机 */
    BSP_POS_ARM_B_FALLING = 1, /**< B 由高变低 -> 制动     */
    BSP_POS_ARM_B_RISING  = 2, /**< B 由低变高 -> 制动     */
} BspPosArm_t;

/**
 * @brief 四路位置 GPIO 的当前电平快照。
 *
 * 所有字段均表示“当前采样时刻是否为低电平有效”，不是历史事件。
 * 历史下降沿必须通过 BSP_PositionGpio_ConsumeFallingEvents() 获取。
 */
typedef struct
{
    bool lockEndActive;     /**< PTA0 当前为低：4WD_LOCK 端点有效 */
    bool fourWdRightActive; /**< PTA1 当前为低：4WD 右位置有效    */
    bool fourWdLeftActive;  /**< PTD3 当前为低：4WD 左位置有效    */
    bool twoWdEndActive;    /**< PTB2 当前为低：2WD 端点有效      */
} BspPositionInputs_t;

/**
 * 清除四路遗留中断标志、读取上电电平、建立端点联锁，
 * 然后使能 GPIOA_IRQn、GPIOB_IRQn 和 GPIOD_IRQn。
 * 调用前必须已经在 main() 中完成 PINS_DRV_Init()。
 */
void BSP_PositionGpio_Init(void);

/**
 * @brief 周期读取四路当前电平并刷新输入快照。
 * @note  建议由 1 ms 任务调用；本函数不清除下降沿历史事件。
 */
void BSP_PositionGpio_Sample(void);

/** 获取最近一次输入快照。 */
const BspPositionInputs_t *BSP_PositionGpio_GetInputs(void);

/** 获取当前低电平有效输入的位掩码。 */
uint8_t BSP_PositionGpio_GetActiveMask(void);

/** 查询模块是否已经完成初始化。 */
bool BSP_PositionGpio_IsInitialized(void);

/**
 * @brief 原子取出并清除自上次调用以来的下降沿事件。
 * @return BSP_POS_EVENT_xxx 的按位或结果；没有事件时返回 0。
 * @note  为避免读改写竞争，本函数会短暂屏蔽 GPIOA/B/D 三个 IRQ。
 */
uint8_t BSP_PositionGpio_ConsumeFallingEvents(void);

/** 清除全部历史下降沿事件。 */
void BSP_PositionGpio_ClearFallingEvents(void);

/* ══════════════════════════════════════════════════════════════════
 *  调试事件日志 —— 问题定位完成后整体删除
 * ══════════════════════════════════════════════════════════════════
 *
 * 环形缓冲，记录 4WD 定位过程中的关键事件。终态快照只能看到"最后停在
 * 哪儿"，看不出中途发生了什么；这个日志把过程留下来，复现后直接导出
 * g_dbgEvt 和 g_dbgIdx 即可还原整条时间线。
 *
 * 读法：从 g_dbgIdx 指向的位置开始是最旧的一条，按 seq 递增排序即为
 * 时间顺序。seq 不连续说明缓冲被绕圈覆盖了。
 */

#define DBG_EVT_N            (64U)

/* tag 取值 */
#define DBG_TAG_ISR_EDGE     (1U)  /**< PTA1 中断，arm = 进入时的 s_arm    */
#define DBG_TAG_ARM          (2U)  /**< 武装，arm = 刚设置的武装状态       */
#define DBG_TAG_TRIGGER      (3U)  /**< 预置触发命中并制动                 */
#define DBG_TAG_ARRIVE       (4U)  /**< ArriveAt，arm = 到达的位置枚举     */
#define DBG_TAG_MOTOR_START  (5U)  /**< 下发运动，arm = BspMotorDir_t      */
#define DBG_TAG_ERROR        (6U)  /**< SetError，arm = 错误码             */
#define DBG_TAG_DIRWAIT      (7U)  /**< 进入换向死区，arm = 死区后的方向   */
#define DBG_TAG_TARGET       (8U)  /**< 目标档位改变，arg = 新的 AppTcPos_t */

/**
 * @brief 单条调试事件。
 *
 * mask 的位定义与 BSP_POS_EVENT_xxx 一致：
 *   bit0 = LOCK 端点   bit1 = 霍尔B(PTA1)
 *   bit2 = 霍尔C(PTD3) bit3 = 2WD 端点
 * 置 1 表示该路当前为低电平有效。mask 由 ReadActiveMaskDirect() 直接
 * 读引脚得到，不经过 task 层的事件合成，因此反映的是真实电平。
 */
typedef struct
{
    uint16_t tag;   /**< DBG_TAG_xxx          */
    uint16_t arg;   /**< 含义随 tag 变化      */
    uint16_t mask;  /**< 事件时刻的真实电平   */
    uint16_t seq;   /**< 递增序号             */
} DbgEvt_t;

extern DbgEvt_t         g_dbgEvt[DBG_EVT_N];
extern volatile uint8_t g_dbgIdx;

/** 记录一条调试事件。可在中断上下文调用。 */
void BSP_DbgLog(uint16_t tag, uint16_t arg);

/* ════════════════════ 调试事件日志结束 ════════════════════ */

/**
 * @brief 武装或解除 4WD 停车预置触发。
 * @param arm 见 BspPosArm_t；传 BSP_POS_ARM_NONE 解除。
 * @note  武装的同时清除上一次的触发标志。判定只由 PTA1 的边沿中断触发，
 *        因此武装瞬间不会命中。
 *        APP 层必须在到位、报错和目标改变时可靠解除，否则执行器之后
 *        路过霍尔B 时会被意外制动。
 */
void BSP_PositionGpio_ArmStop(BspPosArm_t arm);

/** 读取当前武装状态。 */
BspPosArm_t BSP_PositionGpio_GetArm(void);

/**
 * @brief 取出并清除"预置触发已命中"标志。
 * @return true 表示已命中并已完成制动。
 */
bool BSP_PositionGpio_ConsumeStopTrigger(void);

/** 处理 GPIOA 端口中断：PTA0、PTA1。 */
void BSP_PositionGpio_HandlePortAInterrupt(void);

/** 处理 GPIOB 端口中断：PTB2。 */
void BSP_PositionGpio_HandlePortBInterrupt(void);

/** 处理 GPIOD 端口中断：PTD3。 */
void BSP_PositionGpio_HandlePortDInterrupt(void);

/*
 * 默认由本模块提供 GPIOA/B/D_IRQHandler()。
 * 若工程中已有同名中断入口，请定义 BSP_POSITION_GPIO_EXTERNAL_IRQ_HANDLERS，
 * 并分别在现有入口中调用对应的 HandlePortXInterrupt()。
 */
#ifndef BSP_POSITION_GPIO_EXTERNAL_IRQ_HANDLERS
void GPIOA_IRQHandler(void);
void GPIOB_IRQHandler(void);
void GPIOD_IRQHandler(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BSP_POSITION_GPIO_H */