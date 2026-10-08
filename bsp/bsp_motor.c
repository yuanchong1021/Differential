/**
 * @file    bsp_motor.c
 * @brief   双 PWM 电机驱动、端点方向联锁及连续运行超时保护
 *
 * 输出组合（经 ApplyDuty 内部换位后与实测运动方向一致）：
 *   STOP    : 两路占空比 0
 *   FORWARD : ApplyDuty(RUN, 0)      朝 4WD_LOCK 方向
 *   REVERSE : ApplyDuty(0, RUN)      朝 2WD 方向
 *   BRAKE   : ApplyDuty(FULL, FULL)  两路同时为高（需 H 桥支持）
 *
 * 注意：ApplyDuty() 内部把入参 dutyA 写给 CH_PWMB、dutyB 写给 CH_PWMA。
 * 这是按实测运动方向做的换位，不要"顺手改正"，否则方向会反。
 */

#include "bsp_motor.h"
#include "bsp_position_gpio.h"
#include "device_registers.h"

/* 当前实际下发到 PWM 的方向；制动和停机均记为 STOP。 */
static volatile BspMotorDir_t s_currentDir = BSP_MOTOR_STOP;

/* 连续非 STOP 输出的 2 ms 计数，用于独立于 APP 状态机的硬超时保护。 */
static uint32_t               s_runTicks   = 0U;

/* 连续运行超时后的软件锁定标志；锁定时拒绝所有非 STOP 命令。 */
static bool                   s_locked     = false;

/* 两个机械端点的方向联锁。 */
static volatile bool          s_lockEndstopLatched  = false;
static volatile bool          s_twoWdEndstopLatched = false;

/*
 * ── 调试计数器 ──────────────────────────────────────────────────
 *
 * 定位往复运动用，问题解决后可整体删除。
 *
 * 不能拿 s_currentDir 直接比较来数掉头：掉头一定先经过 Brake() 和
 * Stop()，届时 s_currentDir 已经是 STOP，比较恒不成立。因此单独用
 * s_lastNonStopDir 记住上一次真正的运动方向。
 *
 * 同理 Brake() 在 AT_TARGET 和 DIR_CHANGE_WAIT 里是每 2 ms 调用一次
 * 的，直接累加没有意义，这里只统计"从非制动进入制动"的次数。
 */
uint32_t g_dirChangeCount  = 0U; /* FORWARD <-> REVERSE 翻转次数     */
uint32_t g_motorStartCount = 0U; /* 从停止状态起步的次数             */
uint32_t g_brakeEnterCount = 0U; /* 进入制动的次数（非每周期累加）   */

static BspMotorDir_t s_lastNonStopDir = BSP_MOTOR_STOP;
static bool          s_inBrake        = false;

/* 上一次记录的 PWM 输出组合，用于只在变化时记日志。 */
static uint16_t      s_lastPwmCode    = 0xFFFFU;

/**
 * @brief 同时更新 PWMA/PWMB 占空比，并通过 LDOK 一次性生效。
 *
 * ★ 通道换位是刻意的 ★
 * dutyA 写入 CH_PWMB，dutyB 写入 CH_PWMA，与实测运动方向对应。
 */
static void ApplyDuty(uint16_t dutyA, uint16_t dutyB)
{
    /*
     * 临界区。GPIO 中断现在会直接调用 BSP_Motor_Brake() 就地制动，
     * 如果任务上下文正好写到一半被打断，两个通道可能停在不一致的组合上。
     * 三次寄存器写入耗时极短，整段屏蔽中断即可。
     */
    uint32_t primask = __get_PRIMASK();

    MPWM_DRV_UpdateChannelPeriodAndDutyCycle(
        BSP_MOTOR_MPWM_INST,
        BSP_MOTOR_CH_PWMA,
        MPWM_PERIOD_IN_HZ,
        20000U,
        dutyB);

    MPWM_DRV_UpdateChannelPeriodAndDutyCycle(
        BSP_MOTOR_MPWM_INST,
        BSP_MOTOR_CH_PWMB,
        MPWM_PERIOD_IN_HZ,
        20000U,
        dutyA);

    MPWM_DRV_SetLdok(BSP_MOTOR_MPWM_INST, BSP_MOTOR_CH_MASK);


    __set_PRIMASK(primask);
}

/** @brief 建立普通停机输出，并清除本次连续运行计时。 */
static void ApplyStopOutput(void)
{
    ApplyDuty(BSP_MOTOR_STOP_DUTY_A, BSP_MOTOR_STOP_DUTY_B);
    s_currentDir = BSP_MOTOR_STOP;
    s_runTicks   = 0U;
}

void BSP_Motor_Init(void)
{
    MPWM_DRV_EnableCounter(BSP_MOTOR_MPWM_INST, BSP_MOTOR_CH_MASK);

    s_locked              = false;
    s_lockEndstopLatched  = false;
    s_twoWdEndstopLatched = false;
    ApplyStopOutput();
}

/**
 * @brief 请求电机运动方向，速度固定。
 *
 * 安全检查顺序：
 *   1. 若目标方向被端点联锁禁止，保持 STOP；
 *   2. 若连续运行超时锁定且请求不是 STOP，保持 STOP；
 *   3. 检查通过后才更新两个 PWM 通道。
 */
void BSP_Motor_SetDir(BspMotorDir_t dir)
{
    if ((dir == BSP_MOTOR_FORWARD && s_lockEndstopLatched) ||
        (dir == BSP_MOTOR_REVERSE && s_twoWdEndstopLatched))
    {
        ApplyStopOutput();
        return;
    }

    if (s_locked && dir != BSP_MOTOR_STOP)
    {
        ApplyStopOutput();
        return;
    }

    /* ── 调试计数，可随计数器一并删除 ── */
    if (dir == BSP_MOTOR_FORWARD || dir == BSP_MOTOR_REVERSE)
    {
        if (s_currentDir == BSP_MOTOR_STOP)
        {
            g_motorStartCount++;
        }
        if (s_lastNonStopDir != BSP_MOTOR_STOP && s_lastNonStopDir != dir)
        {
            g_dirChangeCount++;
        }
        s_lastNonStopDir = dir;
    }
    s_inBrake = false;
    /* ── 调试计数结束 ── */

    switch (dir)
    {
        case BSP_MOTOR_FORWARD:
            ApplyDuty(BSP_MOTOR_DUTY_RUN, BSP_MOTOR_DUTY_LOW);
            s_currentDir = BSP_MOTOR_FORWARD;
            break;

        case BSP_MOTOR_REVERSE:
            ApplyDuty(BSP_MOTOR_DUTY_LOW, BSP_MOTOR_DUTY_RUN);
            s_currentDir = BSP_MOTOR_REVERSE;
            break;

        case BSP_MOTOR_STOP:
        default:
            ApplyStopOutput();
            break;
    }
}

/**
 * @brief 主动制动。
 *
 * s_currentDir 置为 STOP，使端点联锁和运行超时逻辑把制动视为非运动状态。
 */
void BSP_Motor_Brake(void)
{
    /* ── 调试计数，可随计数器一并删除 ── */
    if (!s_inBrake)
    {
        g_brakeEnterCount++;
        s_inBrake = true;
    }
    /* ── 调试计数结束 ── */

#if (BSP_MOTOR_USE_ACTIVE_BRAKE != 0U)
    ApplyDuty(BSP_MOTOR_DUTY_FULL, BSP_MOTOR_DUTY_FULL);
    s_currentDir = BSP_MOTOR_STOP;
    s_runTicks   = 0U;
#else
    ApplyStopOutput();
#endif
}

BspMotorDir_t BSP_Motor_GetDir(void)
{
    return s_currentDir;
}

/**
 * @brief 根据端点当前电平维护方向联锁。
 *
 * 进入端点立即置位；只有沿安全方向离开并且对应输入已经释放时才清零。
 */
void BSP_Motor_UpdateEndstopInputs(bool atLockEndstop, bool at2WdEndstop)
{
    if (atLockEndstop)
    {
        s_lockEndstopLatched = true;
    }
    else if (s_currentDir == BSP_MOTOR_REVERSE)
    {
        s_lockEndstopLatched = false;
    }

    if (at2WdEndstop)
    {
        s_twoWdEndstopLatched = true;
    }
    else if (s_currentDir == BSP_MOTOR_FORWARD)
    {
        s_twoWdEndstopLatched = false;
    }
}

void BSP_Motor_Stop(void)
{
    s_inBrake = false;   /* 调试计数用，可随计数器一并删除 */
    ApplyStopOutput();
}

void BSP_Motor_Tick(void)
{
    if (s_currentDir == BSP_MOTOR_STOP)
    {
        s_runTicks = 0U;
        return;
    }

    if (s_locked)
    {
        ApplyStopOutput();
        return;
    }

    s_runTicks++;
    if (s_runTicks >= BSP_MOTOR_MAX_RUN_TICKS)
    {
        ApplyStopOutput();
        s_locked   = true;
        s_runTicks = 0U;
    }
}

bool BSP_Motor_IsLocked(void)
{
    return s_locked;
}

void BSP_Motor_ClearLock(void)
{
    s_locked   = false;
    s_runTicks = 0U;
}