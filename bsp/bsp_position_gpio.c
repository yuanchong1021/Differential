/**
 * @file    bsp_position_gpio.c
 * @brief   四路位置 GPIO 当前电平采样、下降沿事件锁存及端点联锁
 *
 * 本文件同时处理两类信息：
 *   1. 当前电平：周期任务读取 GPIO，判断磁铁当前是否仍位于某个检测区；
 *   2. 下降沿历史：GPIO 中断锁存“曾进入过检测区”的事实，防止高速运动时
 *      低脉冲短于任务周期而被轮询遗漏。
 *
 * 端点联锁只针对危险运动方向生效：
 *   - FORWARD 运动时触发 4WD_LOCK 端点，锁存并禁止继续 FORWARD；
 *   - REVERSE 运动时触发 2WD 端点，锁存并禁止继续 REVERSE；
 *   - ISR 不直接更新 PWM，实际普通停机由 Task_Ctrl_Run() 在任务上下文执行。
 *
 * ── PTA1 判沿方式 ──────────────────────────────────────────────────
 *
 * PTA1 的 IRQC 由 BSP_PositionGpio_ArmStop() 动态切换，硬件只在武装的那个
 * 沿上产生中断，ISR 因此不需要回读电平来判断这次是哪个边沿。
 *
 * 旧实现用“清标志后回读电平”分类边沿，该方法不可靠：PTA1 开了数字滤波器，
 * PDIR 相对边沿检测存在传播延迟；GPIOA_IRQn 又会被 ConsumeStopTrigger() 和
 * ConsumeFallingEvents() 周期性屏蔽，ApplyDuty() 还会关 PRIMASK。ISR 延迟
 * 执行时回读值与触发它的边沿脱钩，会把下降沿误判成上升沿——表现为 4WD 停车
 * 点整体偏移一个检测区宽度。
 *
 * 未武装时 PTA1 固定为下降沿，只承担 s_fallingEventMask 的事件锁存职责。
 * 武装 ARM_B_RISING 期间这一趟拿不到 B 的下降沿事件；HandleSeek4WD() 只看
 * ConsumeStopTrigger()，Is4WdPositionActive() 用的是 Sample() 的电平快照，
 * 因此没有实际影响。
 *
 * 注意：PINS_DRV_Init() 已在 main.c 中完成。本文件不会重复配置引脚复用、
 * 输入方向或上下拉，只管理 PTA1 的中断触发类型、中断标志、NVIC 和输入数据。
 */

#include "bsp_position_gpio.h"
#include "bsp_motor.h"
#include "device_registers.h"
#include "interrupt_manager.h"


/* ── 定位用，问题解决后整体删除 ────────────────────────────── */
volatile uint32_t g_dbgIrqcAtArm  = 0U;  /* 武装瞬间 GPIOA->PCR[1] 的 IRQC */
volatile uint16_t g_dbgIsrCount   = 0U;  /* 本次武装后 PTA1 进中断次数     */
volatile uint16_t g_dbgIsrLevels  = 0U;  /* 每次回读的 bLow，逐位左移      */
volatile uint8_t  g_dbgArmAtIsr[8];      /* 每次进中断时的 s_arm           */
volatile uint32_t g_dbgPdirAtBrake = 0U; /* 制动那一刻的 GPIOA->PDIR       */

/*
 * 下降沿事件锁存位：由 GPIOA/B/D 中断置位，由主循环任务读取并清零。
 * 使用 32 位且自然对齐，是为了让 Cortex-M0+ 对该变量的单次读写保持原子性；
 * 但“读取后清零”仍是两个操作，因此 Consume 接口会短暂屏蔽相关 IRQ。
 */
static volatile uint32_t   s_fallingEventMask = 0U;

/* 初始化完成标志。未完成初始化时，周期采样函数直接返回。 */
static bool                s_initialized      = false;

/* 四路输入的最新电平快照，仅表示当前状态，不包含历史下降沿。 */
static BspPositionInputs_t s_inputs = { false, false, false, false };

/*
 * 4WD 停车预置触发。
 *
 * s_arm 由 APP 层武装；武装的同时 PTA1 的 IRQC 被切换到对应的单一边沿。
 * 命中时立即制动并置位 s_stopTriggered。周期采样不参与判定。
 */
static volatile BspPosArm_t s_arm           = BSP_POS_ARM_NONE;
static volatile bool        s_stopTriggered = false;

/**
 * @brief 读取单个位置输入是否有效。
 * @param gpio GPIO 外设基址，只能传 GPIOA/GPIOB/GPIOD 等 GPIO_Type 指针。
 * @param pin  端口内的引脚序号。
 * @return true 表示引脚当前为低电平，即磁铁位于对应检测范围内。
 *
 * 本项目位置输入采用低电平有效，因此 GPIO 读值为 0 时返回 true。
 */
static bool InputIsActive(GPIO_Type *gpio, uint32_t pin)
{
    return (PINS_DRV_ReadPin(gpio, pin) == 0U);
}

/**
 * @brief 直接读取四路 GPIO，并压缩成统一位掩码。
 * @return BSP_POS_EVENT_xxx 的按位或结果。
 *
 * 这里直接访问硬件而不是读取 s_inputs，主要用于初始化和周期采样入口。
 * 函数名称中的 Direct 表示“直接读引脚”，并不表示绕过驱动寄存器操作。
 */
static uint8_t ReadActiveMaskDirect(void)
{
    uint8_t mask = BSP_POS_EVENT_NONE;

    if (InputIsActive(BSP_POS_LOCK_END_GPIO, BSP_POS_LOCK_END_PIN))
    {
        mask |= BSP_POS_EVENT_LOCK_END;
    }
    if (InputIsActive(BSP_POS_4WD_RIGHT_GPIO, BSP_POS_4WD_RIGHT_PIN))
    {
        mask |= BSP_POS_EVENT_4WD_RIGHT;
    }
    if (InputIsActive(BSP_POS_4WD_LEFT_GPIO, BSP_POS_4WD_LEFT_PIN))
    {
        mask |= BSP_POS_EVENT_4WD_LEFT;
    }
    if (InputIsActive(BSP_POS_2WD_END_GPIO, BSP_POS_2WD_END_PIN))
    {
        mask |= BSP_POS_EVENT_2WD_END;
    }

    return mask;
}

/* ══════════════════════════════════════════════════════════════════
 *  调试事件日志实现 —— 问题定位完成后整体删除
 * ══════════════════════════════════════════════════════════════════ */

DbgEvt_t         g_dbgEvt[DBG_EVT_N];
volatile uint8_t g_dbgIdx  = 0U;
static  uint16_t s_dbgSeq  = 0U;

/**
 * @brief 记录一条调试事件。
 *
 * 会在中断上下文调用，因此整段屏蔽中断保证索引和内容一致。
 * mask 直接读引脚，不走 s_inputs，也不经过 task 层的事件合成，
 * 这样记录的是事件发生瞬间的真实电平。
 *
 * 注意 mask 的语义与 PDIR 相反：置 1 表示该路当前为低电平有效。
 */
void BSP_DbgLog(uint16_t tag, uint16_t arg)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t  i;

    __disable_irq();

    i = g_dbgIdx;
    g_dbgEvt[i].tag  = tag;
    g_dbgEvt[i].arg  = arg;
    g_dbgEvt[i].mask = ReadActiveMaskDirect();
    g_dbgEvt[i].seq  = s_dbgSeq++;
    g_dbgIdx = (uint8_t)((i + 1U) % DBG_EVT_N);

    __set_PRIMASK(primask);
}

/* ════════════════════ 调试事件日志实现结束 ════════════════════ */

/**
 * @brief 根据位掩码刷新结构化输入快照。
 *
 * 将位运算集中在此函数中，可保证 GetInputs() 返回的四个布尔字段始终
 * 与 GetActiveMask() 使用相同的位定义，避免位置映射不一致。
 */
static void UpdateInputsFromMask(uint8_t mask)
{
    s_inputs.lockEndActive =
        ((mask & BSP_POS_EVENT_LOCK_END) != 0U);
    s_inputs.fourWdRightActive =
        ((mask & BSP_POS_EVENT_4WD_RIGHT) != 0U);
    s_inputs.fourWdLeftActive =
        ((mask & BSP_POS_EVENT_4WD_LEFT) != 0U);
    s_inputs.twoWdEndActive =
        ((mask & BSP_POS_EVENT_2WD_END) != 0U);
}

/**
 * 清除指定 GPIO 引脚的中断标志。
 * 注意：YTM32 pins_driver 的端口入参是 GPIO_Type *，不是 PORT_Type/PCTRLx。
 */
static void ClearPendingFlag(GPIO_Type *gpio, uint32_t pin)
{
    PINS_DRV_ClearPinIntFlagCmd(gpio, pin);
}

/**
 * @brief 按武装状态设置 PTA1 的中断触发沿。
 * @param arm 见 BspPosArm_t。
 *
 * 只能在 GPIOA_IRQn 已被屏蔽时调用。切换 IRQC 的动作本身可能置出伪标志，
 * 因此切换后必须清 PIFR，否则重新使能 NVIC 会立刻进一次假中断并误制动。
 */
static void SetHallBEdge(BspPosArm_t arm)
{
    gpio_interrupt_config_t edge;

    switch (arm)
    {
        case BSP_POS_ARM_B_FALLING:
            edge = PCTRL_INT_FALLING_EDGE;
            break;

        case BSP_POS_ARM_B_RISING:
            edge = PCTRL_INT_RISING_EDGE;
            break;

        case BSP_POS_ARM_NONE:
        default:
            /* 未武装：下降沿，只用于 s_fallingEventMask 的事件锁存。 */
            edge = PCTRL_INT_FALLING_EDGE;
            break;
    }

    PINS_DRV_SetPinIntSel(BSP_POS_4WD_RIGHT_GPIO,
                          BSP_POS_4WD_RIGHT_PIN,
                          edge);

    /* 丢掉切换过程中可能产生的伪标志。 */
    PINS_DRV_ClearPinIntFlagCmd(BSP_POS_4WD_RIGHT_GPIO,
                                BSP_POS_4WD_RIGHT_PIN);
}

/**
 * @brief 暂时屏蔽所有与位置输入有关的端口中断。
 *
 * PTA0/PTA1 共用 GPIOA_IRQn，PTB2 使用 GPIOB_IRQn，PTD3 使用 GPIOD_IRQn。
 * 该函数只操作 NVIC 使能，不会改变各引脚的触发沿配置。
 */
static void DisablePositionInterrupts(void)
{
    INT_SYS_DisableIRQ(GPIOA_IRQn);
    INT_SYS_DisableIRQ(GPIOB_IRQn);
    INT_SYS_DisableIRQ(GPIOD_IRQn);

    /* Cortex-M0+ 上确保屏蔽在后续访问共享变量之前真正生效。 */
    __DSB();
    __ISB();
}

/** @brief 恢复使能三个位置 GPIO 端口中断。 */
static void EnablePositionInterrupts(void)
{
    INT_SYS_EnableIRQ(GPIOA_IRQn);
    INT_SYS_EnableIRQ(GPIOB_IRQn);
    INT_SYS_EnableIRQ(GPIOD_IRQn);
}

/**
 * 处理已经识别出的下降沿事件。
 * 仅锁存事件并更新端点方向联锁；不在 GPIO 中断中操作 PWM。
 */
static void ProcessHitMask(uint8_t hitMask)
{
    bool          hitLockEnd;
    bool          hit2WdEnd;
    bool          lockEndLevelActive;
    bool          twoWdEndLevelActive;
    BspMotorDir_t direction;

    if (hitMask == BSP_POS_EVENT_NONE)
    {
        return;
    }

    /*
     * 先锁存事件，再进行其他处理。即使后续端点停机或任务尚未运行，
     * TASK 层仍能在下一控制周期取得该下降沿，不会因为脉冲已经释放而丢失。
     */
    s_fallingEventMask |= hitMask;

    /* 两个 4WD 位置点只需要锁存；机械端点还需要按运动方向建立联锁。 */
    hitLockEnd = ((hitMask & BSP_POS_EVENT_LOCK_END) != 0U);
    hit2WdEnd  = ((hitMask & BSP_POS_EVENT_2WD_END) != 0U);
    direction  = BSP_Motor_GetDir();

    lockEndLevelActive =
        InputIsActive(BSP_POS_LOCK_END_GPIO, BSP_POS_LOCK_END_PIN);
    twoWdEndLevelActive =
        InputIsActive(BSP_POS_2WD_END_GPIO, BSP_POS_2WD_END_PIN);

    /*
     * 当前端点电平有效时无条件建立联锁。
     * 对已经结束的窄脉冲，仅在电机正朝该端点运动时建立联锁。
     */
    BSP_Motor_UpdateEndstopInputs(
        (bool)(lockEndLevelActive ||
               (hitLockEnd && (direction == BSP_MOTOR_FORWARD))),
        (bool)(twoWdEndLevelActive ||
               (hit2WdEnd && (direction == BSP_MOTOR_REVERSE))));

    /*
     * GPIO ISR 中只完成事件锁存和端点状态更新。
     * 不在中断里直接调用 PWM 更新函数，避免驱动层寄存器操作不满足 ISR 安全要求。
     * 实际普通停机由 Task_Ctrl_Run() 在任务上下文执行。
     */
}

/**
 * @brief 初始化位置 GPIO 事件模块。
 *
 * 调用顺序要求：
 *   1. main.c 已执行 PINS_DRV_Init()；
 *   2. PWM 已初始化，BSP_Motor_Init() 已建立 LOW/LOW 普通停机；
 *   3. 再调用本函数清标志、读取上电电平并使能位置中断。
 *
 * 先禁用 IRQ、清除遗留标志，再读取当前电平，避免初始化期间的旧标志被
 * 当成真实运动事件。上电已经压住端点时不会产生新下降沿，因此必须在
 * 此处读取电平并建立端点方向联锁。
 */
void BSP_PositionGpio_Init(void)
{
    uint8_t activeMask;

    s_initialized      = false;
    s_fallingEventMask = 0U;
    s_arm              = BSP_POS_ARM_NONE;
    s_stopTriggered    = false;

    /* 三个端口分别对应独立的 NVIC IRQ。 */
    DisablePositionInterrupts();

    /*
     * 引脚 MUX、方向和上下拉已经由 main() 中的 PINS_DRV_Init() 统一配置。
     * PTA1 的触发沿由本模块接管：未武装状态固定为下降沿。
     */
    SetHallBEdge(BSP_POS_ARM_NONE);

    ClearPendingFlag(BSP_POS_LOCK_END_GPIO, BSP_POS_LOCK_END_PIN);
    ClearPendingFlag(BSP_POS_4WD_RIGHT_GPIO, BSP_POS_4WD_RIGHT_PIN);
    ClearPendingFlag(BSP_POS_4WD_LEFT_GPIO, BSP_POS_4WD_LEFT_PIN);
    ClearPendingFlag(BSP_POS_2WD_END_GPIO, BSP_POS_2WD_END_PIN);

    /*
     * PIFR 清零不会清 NVIC 的 pending 位。上电或复位遗留的 pending 会在
     * 使能瞬间触发一次空 ISR，这里一并清掉。
     */
    INT_SYS_ClearPending(GPIOA_IRQn);
    INT_SYS_ClearPending(GPIOB_IRQn);
    INT_SYS_ClearPending(GPIOD_IRQn);

    activeMask = ReadActiveMaskDirect();
    UpdateInputsFromMask(activeMask);

    /*
     * 上电时端点可能已经为低，不会产生下降沿。
     * 直接用当前电平建立方向联锁，避免继续向端点外运动。
     */
    BSP_Motor_UpdateEndstopInputs(s_inputs.lockEndActive,
                                  s_inputs.twoWdEndActive);

    /* 设置GPIO中断优先级 */
    INT_SYS_SetPriority(GPIOA_IRQn, 0U);
    INT_SYS_SetPriority(GPIOB_IRQn, 0U);
    INT_SYS_SetPriority(GPIOD_IRQn, 0U);

    s_initialized = true;
    EnablePositionInterrupts();
}

/* 调试用：原始 PDIR 快照，1 = 高电平。与 s_inputs 的语义相反。 */
volatile uint32_t g_dbgPdirA;   /* bit0 = PTA0, bit1 = PTA1 */
volatile uint32_t g_dbgPdirB;   /* bit2 = PTB2 */
volatile uint32_t g_dbgPdirD;   /* bit3 = PTD3 */

void BSP_PositionGpio_Sample(void)
{
    uint8_t activeMask;

    if (!s_initialized) { return; }

    g_dbgPdirA = GPIOA->PDIR;
    g_dbgPdirB = GPIOB->PDIR;
    g_dbgPdirD = GPIOD->PDIR;

    activeMask = ReadActiveMaskDirect();
    UpdateInputsFromMask(activeMask);
    BSP_Motor_UpdateEndstopInputs(s_inputs.lockEndActive,
                                  s_inputs.twoWdEndActive);
}

/**
 * @brief 返回最新位置输入快照的只读指针。
 * @note  调用者不应修改返回对象；快照在下一次 Sample() 时更新。
 */
const BspPositionInputs_t *BSP_PositionGpio_GetInputs(void)
{
    return &s_inputs;
}

/** @brief 把最新结构化输入快照重新编码为统一位掩码。 */
uint8_t BSP_PositionGpio_GetActiveMask(void)
{
    uint8_t mask = BSP_POS_EVENT_NONE;

    if (s_inputs.lockEndActive)
    {
        mask |= BSP_POS_EVENT_LOCK_END;
    }
    if (s_inputs.fourWdRightActive)
    {
        mask |= BSP_POS_EVENT_4WD_RIGHT;
    }
    if (s_inputs.fourWdLeftActive)
    {
        mask |= BSP_POS_EVENT_4WD_LEFT;
    }
    if (s_inputs.twoWdEndActive)
    {
        mask |= BSP_POS_EVENT_2WD_END;
    }

    return mask;
}

bool BSP_PositionGpio_IsInitialized(void)
{
    return s_initialized;
}

/**
 * @brief 原子取出并清除累计的下降沿事件。
 *
 * 三个端口中断均可能在任意时刻置位 s_fallingEventMask。如果直接执行
 * “读取变量；变量清零”，可能在两条语句之间插入 ISR，导致新事件被清掉。
 * 因此本函数在极短临界区内屏蔽 GPIOA/B/D IRQ，完成读出和清零后立即恢复。
 */
uint8_t BSP_PositionGpio_ConsumeFallingEvents(void)
{
    uint8_t events;

    /* 防止三个端口 ISR 在读取与清零之间插入导致事件丢失。 */
    DisablePositionInterrupts();
    events = (uint8_t)(s_fallingEventMask & BSP_POS_EVENT_ALL);
    s_fallingEventMask = 0U;
    EnablePositionInterrupts();

    return events;
}

/**
 * @brief 丢弃所有尚未消费的下降沿事件。
 * @note  主要用于上电稳定阶段结束时，防止初始化抖动被当作运动事件。
 */
void BSP_PositionGpio_ClearFallingEvents(void)
{
    DisablePositionInterrupts();
    s_fallingEventMask = 0U;
    EnablePositionInterrupts();
}

/**
 * @brief 武装或解除 4WD 停车预置触发。
 *
 * 除了设置 s_arm，还会把 PTA1 的 IRQC 切换到要等的那个沿，使中断只可能
 * 由目标边沿产生。ISR 因此不需要、也不应该再回读电平判断边沿方向。
 *
 * 切换 IRQC 和清 PIFR 必须在 GPIOA_IRQn 屏蔽期间完成，否则切换瞬间的
 * 伪标志会立刻触发一次误制动。
 */
void BSP_PositionGpio_ArmStop(BspPosArm_t arm)
{
    INT_SYS_DisableIRQ(GPIOA_IRQn);
    __DSB();
    __ISB();

    s_arm           = arm;
        g_dbgIrqcAtArm = GPIOA->PCR[1] & 0xFU;
    g_dbgIsrCount  = 0U;
    g_dbgIsrLevels = 0U;
    s_stopTriggered = false;

    SetHallBEdge(arm);

    INT_SYS_EnableIRQ(GPIOA_IRQn);

    //BSP_DbgLog(DBG_TAG_ARM, (uint16_t)arm); /* 调试 */
}

BspPosArm_t BSP_PositionGpio_GetArm(void)
{
    return s_arm;
}

bool BSP_PositionGpio_ConsumeStopTrigger(void)
{
    bool triggered;

    INT_SYS_DisableIRQ(GPIOA_IRQn);
    __DSB();
    __ISB();

    triggered       = s_stopTriggered;
    s_stopTriggered = false;

    INT_SYS_EnableIRQ(GPIOA_IRQn);

    return triggered;
}

/**
 * @brief GPIOA 端口中断处理：识别 PTA0 和 PTA1 的中断标志。
 *
 * 一个端口中断入口可能同时包含多个引脚标志，因此必须读取整个端口标志
 * 后逐位判断，并且只清除本模块负责的引脚，避免影响 GPIOA 上其他业务。
 */
void BSP_PositionGpio_HandlePortAInterrupt(void)
{
    uint32_t flags;
    uint8_t  hitMask = BSP_POS_EVENT_NONE;

    /* pins_driver 要求传 GPIOA，而不是 PCTRLA。 */
    flags = PINS_DRV_GetPortIntFlag(GPIOA);

    if ((flags & (1UL << BSP_POS_LOCK_END_PIN)) != 0UL)
    {
        PINS_DRV_ClearPinIntFlagCmd(BSP_POS_LOCK_END_GPIO,
                                    BSP_POS_LOCK_END_PIN);
        hitMask |= BSP_POS_EVENT_LOCK_END;
    }

    if ((flags & (1UL << BSP_POS_4WD_RIGHT_PIN)) != 0UL)
    {
        PINS_DRV_ClearPinIntFlagCmd(BSP_POS_4WD_RIGHT_GPIO,
                                    BSP_POS_4WD_RIGHT_PIN);

        if (s_arm != BSP_POS_ARM_NONE)
        {
            /*
             * ★ 硬件保证只在武装的那个沿进来，不再回读电平 ★
             *
             * ArmStop() 已把 IRQC 设成 FALLING 或 RISING，因此走到这里
             * 就意味着目标边沿已经发生。回读电平会受数字滤波器延迟和
             * 中断屏蔽窗口影响，拿到的可能是跳变前的旧值，据此判沿会把
             * 下降沿误判成上升沿，使停车点偏移一个检测区宽度。
             *
             * 制动放在最前面，使停机尽可能贴近边沿时刻。
             */
            s_arm = BSP_POS_ARM_NONE;
            BSP_Motor_Brake();
            s_stopTriggered = true;

            BSP_DbgLog(DBG_TAG_TRIGGER, 0U); /* 调试 */
        }
        else
        {
            /*
             * 未武装：当前是下降沿配置，本次中断必定是进入检测区。
             *
             * s_fallingEventMask 的语义是“曾经进入过检测区”，只有低电平
             * 才算数。武装 ARM_B_RISING 期间不产生下降沿事件，这一趟的
             * 进区信息由 Sample() 的电平快照承担。
             */
            hitMask |= BSP_POS_EVENT_4WD_RIGHT;
        }
    }

    ProcessHitMask(hitMask);
}

/** @brief GPIOB 端口中断处理：识别 PTB2（2WD 端点）下降沿。 */
void BSP_PositionGpio_HandlePortBInterrupt(void)
{
    uint32_t flags;
    uint8_t  hitMask = BSP_POS_EVENT_NONE;

    flags = PINS_DRV_GetPortIntFlag(GPIOB);

    if ((flags & (1UL << BSP_POS_2WD_END_PIN)) != 0UL)
    {
        PINS_DRV_ClearPinIntFlagCmd(BSP_POS_2WD_END_GPIO,
                                    BSP_POS_2WD_END_PIN);
        hitMask |= BSP_POS_EVENT_2WD_END;
    }

    ProcessHitMask(hitMask);
}

/** @brief GPIOD 端口中断处理：识别 PTD3（4WD 左位置）下降沿。 */
void BSP_PositionGpio_HandlePortDInterrupt(void)
{
    uint32_t flags;
    uint8_t  hitMask = BSP_POS_EVENT_NONE;

    flags = PINS_DRV_GetPortIntFlag(GPIOD);

    if ((flags & (1UL << BSP_POS_4WD_LEFT_PIN)) != 0UL)
    {
        PINS_DRV_ClearPinIntFlagCmd(BSP_POS_4WD_LEFT_GPIO,
                                    BSP_POS_4WD_LEFT_PIN);
        hitMask |= BSP_POS_EVENT_4WD_LEFT;
    }

    ProcessHitMask(hitMask);
}

/*
 * 默认中断入口。若工程其他模块已经定义同名 Handler，应在编译选项中定义
 * BSP_POSITION_GPIO_EXTERNAL_IRQ_HANDLERS，并从工程现有 Handler 调用上面的
 * HandlePortXInterrupt()，避免链接阶段出现重复符号。
 */
#ifndef BSP_POSITION_GPIO_EXTERNAL_IRQ_HANDLERS
void GPIOA_IRQHandler(void)
{
    BSP_PositionGpio_HandlePortAInterrupt();
}

void GPIOB_IRQHandler(void)
{
    BSP_PositionGpio_HandlePortBInterrupt();
}

void GPIOD_IRQHandler(void)
{
    BSP_PositionGpio_HandlePortDInterrupt();
}
#endif