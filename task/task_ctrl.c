/**
 * @file    task_ctrl.c
 * @brief   输入汇总、分动器状态机调度及输出更新
 *
 * 本任务是 TASK 层的数据汇合点，不直接读取 GPIO 寄存器：位置数据来自
 * task_position，目标档位来自 task_switch，控制决策交给 app_tc。
 */

#include "task_ctrl.h"
#include "app_tc.h"
#include "task_position.h"
#include "task_switch.h"
#include "bsp_indicator.h"
#include "bsp_motor.h"
#include <stdbool.h>

/* 分动器应用状态机的唯一上下文实例。 */
static AppTcCtx_t s_tcCtx;

/* 确保上电稳定位置只预加载一次，避免运行中反复覆盖状态机历史位置。 */
static bool       s_positionPreloaded = false;

/** @brief 将 APP 层位置枚举转换为板级两线指示编码。 */
static BspIndState_t PosToInd(AppTcPos_t position)
{
    switch (position)
    {
        case APP_TC_POS_2WD:
            return BSP_IND_2WD;

        case APP_TC_POS_4WD:
            return BSP_IND_4WD;

        case APP_TC_POS_4WD_LOCK:
            return BSP_IND_4WD_LOCK;

        default:
            return BSP_IND_NONE;
    }
}

/**
 * @brief 初始化控制任务状态。
 * @note  PWM 和 BSP_Motor_Init() 已在 Board_Init() 中完成，这里再次 Stop 是
 *        防御性操作，确保进入调度循环前输出仍为普通停机。
 */
void Task_Ctrl_Init(void)
{
    /* BSP_Motor_Init 已在 Board_Init 的 PWM 初始化后执行。 */
    BSP_Motor_Stop();
    BSP_Indicator_Init();

    APP_TC_Init(&s_tcCtx);
    s_positionPreloaded = false;
}

/** @brief 1 ms 周期控制入口。 */
void Task_Ctrl_Run(void)
{
    BspPositionInputs_t positionInputs;
    uint8_t             fallingEvents;
    AppSwState_t        switchState;

    /*
     * 位置输入尚未稳定时，禁止任何状态机动作。指示输出也置为 NONE，避免
     * 对外报告一个尚未确认的档位。
     */
    if (!Task_Position_IsReady())
    {
        BSP_Motor_Stop();
        BSP_Indicator_Set(BSP_IND_NONE);
        return;
    }

    positionInputs = *Task_Position_GetInputs();

    /*
     * 只在四路 GPIO 完成上电稳定确认后预加载一次位置。
     * 上电已经处于任一目标位置时，先识别位置，再运行状态机，不会先微动。
     */
    if (!s_positionPreloaded)
    {
        APP_TC_PreloadPos(&s_tcCtx, &positionInputs);
        (void)Task_Position_ConsumeFallingEvents();
        s_positionPreloaded = true;
    }

    fallingEvents = Task_Position_ConsumeFallingEvents();

    /*
     * GPIO 中断只负责锁存下降沿，不直接操作 PWM。
     * 在任务上下文检测端点事件后执行普通停机，保证电机驱动接口安全调用。
     */
    if ((fallingEvents & BSP_POS_EVENT_LOCK_END) != 0U ||
        (fallingEvents & BSP_POS_EVENT_2WD_END) != 0U)
    {
        BSP_Motor_Stop();
    }

    /*
     * 两个控制周期之间出现过的短低脉冲，在当前周期合成为一次有效输入。
     * 对于 4WD 目标，PTD3/PTA1 的下降沿事件会在本周期直接触发到位停机，
     * 不再等待对应 GPIO 恢复为高电平。
     */
    if ((fallingEvents & BSP_POS_EVENT_LOCK_END) != 0U)
    {
        positionInputs.lockEndActive = true;
    }
    if ((fallingEvents & BSP_POS_EVENT_4WD_RIGHT) != 0U)
    {
        positionInputs.fourWdRightActive = true;
    }
    if ((fallingEvents & BSP_POS_EVENT_4WD_LEFT) != 0U)
    {
        positionInputs.fourWdLeftActive = true;
    }
    if ((fallingEvents & BSP_POS_EVENT_2WD_END) != 0U)
    {
        positionInputs.twoWdEndActive = true;
    }

    switchState = Task_Switch_GetState();
    APP_TC_Update(&s_tcCtx, &positionInputs, switchState);

    /* APP 决策完成后再更新独立运行超时保护。 */
    BSP_Motor_Tick();

    /* 仅 AT_TARGET 状态对外输出位置；运动中和错误状态输出 NONE。 */
    BSP_Indicator_Set(PosToInd(APP_TC_GetArrivedPos(&s_tcCtx)));
}

bool Task_Ctrl_IsAtTarget(void)
{
    return APP_TC_IsAtTarget(&s_tcCtx);
}

bool Task_Ctrl_IsError(void)
{
    return APP_TC_IsError(&s_tcCtx);
}

void Task_Ctrl_Reset(void)
{
    APP_TC_Reset(&s_tcCtx);
    BSP_Motor_ClearLock();
}
