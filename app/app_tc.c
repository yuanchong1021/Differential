/**
 * @file    app_tc.c
 * @brief   分动器档位控制状态机
 *
 * 结构：
 *   APP_TC_Update()  每 2 ms 一次：刷新位置 -> 处理档位变化 -> 分发状态
 *   Replan()         按当前目标决定下一步动作，不含任何等待
 *   RequestMotion()  下发方向；仅掉头时插入换向死区
 *
 * 4WD 的停车判定不在本层。BSP 层的预置触发在 PTA1 中断里直接制动，
 * 本层只在 HandleSeek4WD() 里取走触发标志并完成状态迁移。
 */

#include "app_tc.h"
#include <string.h>



static AppTcPos_t SwToPos(AppSwState_t swState);
static bool Is4WdPositionActive(const BspPositionInputs_t *inputs);
static bool AlreadyAtTarget(const AppTcCtx_t *ctx,
                            const BspPositionInputs_t *inputs);
static BspMotorDir_t ChooseDirection(const AppTcCtx_t *ctx,
                                     const BspPositionInputs_t *inputs);
static void ArmForSeek(BspMotorDir_t dir);
static void RequestMotion(AppTcCtx_t *ctx,
                          BspMotorDir_t newDirection,
                          AppTcState_t nextState);
static void Replan(AppTcCtx_t *ctx, const BspPositionInputs_t *inputs);
static void ArriveAt(AppTcCtx_t *ctx, AppTcPos_t position);
static void SetError(AppTcCtx_t *ctx, AppTcErrCode_t errorCode);
static void HandleDirectionWait(AppTcCtx_t *ctx);
static void HandleHoming(AppTcCtx_t *ctx, const BspPositionInputs_t *inputs);
static void HandleMoving(AppTcCtx_t *ctx, const BspPositionInputs_t *inputs);
static void HandleSeek4WD(AppTcCtx_t *ctx, const BspPositionInputs_t *inputs);

/* ── 初始化 ─────────────────────────────────────────────────────── */

void APP_TC_Init(AppTcCtx_t *ctx)
{
    memset(ctx, 0, sizeof(AppTcCtx_t));

    ctx->state          = APP_TC_STATE_IDLE;
    ctx->stateAfterWait = APP_TC_STATE_IDLE;
    ctx->target         = APP_TC_POS_NONE;
    ctx->lastKnownPos   = APP_TC_POS_NONE;
    ctx->motionDir      = BSP_MOTOR_STOP;
    ctx->lastMotionDir  = BSP_MOTOR_STOP;
    ctx->seekDir        = BSP_MOTOR_STOP;
    ctx->dirAfterWait   = BSP_MOTOR_STOP;
    ctx->lastHall       = APP_TC_HALL_NONE;

    BSP_PositionGpio_ArmStop(BSP_POS_ARM_NONE);
}

/**
 * @brief 按上电稳定电平预加载位置。
 *
 * 上电时 GPIO 可能一开始就是低电平，不产生边沿，因此必须按电平判断，
 * 否则会出现"已经在目标点却先启动电机"。
 *
 * 霍尔B 被压住说明位于合法的 4WD 落点，直接置 atFourWd，避免停在 4WD
 * 时重新上电又动一次。
 */
void APP_TC_PreloadPos(AppTcCtx_t *ctx,
                       const BspPositionInputs_t *positionInputs)
{
    if (positionInputs->lockEndActive)
    {
        ctx->lastKnownPos = APP_TC_POS_4WD_LOCK;
        ctx->homingDone   = true;
    }
    else if (positionInputs->twoWdEndActive)
    {
        ctx->lastKnownPos = APP_TC_POS_2WD;
        ctx->homingDone   = true;
    }
    else if (Is4WdPositionActive(positionInputs))
    {
        ctx->lastKnownPos = APP_TC_POS_4WD;
        ctx->homingDone   = true;
    }
    else
    {
        /* 四路全高：位置未知，需要归位。 */
        ctx->lastKnownPos = APP_TC_POS_NONE;
        ctx->homingDone   = false;
    }

    if (positionInputs->fourWdRightActive)
    {
        ctx->lastHall = APP_TC_HALL_B;
    }
    else if (positionInputs->fourWdLeftActive)
    {
        ctx->lastHall = APP_TC_HALL_C;
    }

    ctx->atFourWd = positionInputs->fourWdRightActive;

    BSP_PositionGpio_ArmStop(BSP_POS_ARM_NONE);
}
volatile uint8_t g_dbgTgtHist[8];
volatile uint8_t g_dbgTgtIdx = 0U;
/* ── 主入口 ─────────────────────────────────────────────────────── */

void APP_TC_Update(AppTcCtx_t *ctx,
                   const BspPositionInputs_t *positionInputs,
                   AppSwState_t swState)
{
    AppTcPos_t newTarget;

    /* 端点优先级高于 4WD 检测区，兼容磁场范围重叠。 */
    if (positionInputs->lockEndActive)
    {
        ctx->lastKnownPos = APP_TC_POS_4WD_LOCK;
        ctx->homingDone   = true;
    }
    else if (positionInputs->twoWdEndActive)
    {
        ctx->lastKnownPos = APP_TC_POS_2WD;
        ctx->homingDone   = true;
    }
    else if (Is4WdPositionActive(positionInputs))
    {
        ctx->lastKnownPos = APP_TC_POS_4WD;
        ctx->homingDone   = true;
    }

    if (positionInputs->fourWdRightActive)
    {
        ctx->lastHall = APP_TC_HALL_B;
    }
    else if (positionInputs->fourWdLeftActive)
    {
        ctx->lastHall = APP_TC_HALL_C;
    }

    newTarget = SwToPos(swState);

    /* 档位改变：立即按新目标重新规划。 */
    if (newTarget != APP_TC_POS_NONE && newTarget != ctx->target)
    {
        //BSP_DbgLog(DBG_TAG_TARGET, (uint16_t)newTarget); /* 调试 */
        if (newTarget != APP_TC_POS_NONE && newTarget != ctx->target)
        {
            if (g_dbgTgtIdx < 8U) 
            { g_dbgTgtHist[g_dbgTgtIdx++] = (uint8_t)newTarget; }
        }
        ctx->target    = newTarget;
        ctx->tickCount = 0U;
        ctx->errorCode = APP_TC_ERR_NONE;

        /* 目标不再是 4WD 时，之前的定位结果失效。 */
        if (newTarget != APP_TC_POS_4WD)
        {
            ctx->atFourWd = false;
        }

        BSP_PositionGpio_ArmStop(BSP_POS_ARM_NONE);

        /* 允许从 15 s 连续运行硬锁定中恢复。 */
        BSP_Motor_ClearLock();

        /* 换向死区未走完时不打断，结束后回 IDLE 重新规划。 */
        if (ctx->state == APP_TC_STATE_DIR_CHANGE_WAIT)
        {
            ctx->stateAfterWait = APP_TC_STATE_IDLE;
            ctx->dirAfterWait   = BSP_MOTOR_STOP;
        }
        else
        {
            Replan(ctx, positionInputs);
        }
    }

    if (ctx->target == APP_TC_POS_NONE)
    {
        BSP_Motor_Stop();
        ctx->motionDir = BSP_MOTOR_STOP;
        return;
    }

    switch (ctx->state)
    {
        case APP_TC_STATE_IDLE:
            Replan(ctx, positionInputs);
            break;

        case APP_TC_STATE_HOMING:
            HandleHoming(ctx, positionInputs);
            break;

        case APP_TC_STATE_DIR_CHANGE_WAIT:
            HandleDirectionWait(ctx);
            break;

        case APP_TC_STATE_MOVING:
            HandleMoving(ctx, positionInputs);
            break;

        case APP_TC_STATE_SEEK_4WD:
            HandleSeek4WD(ctx, positionInputs);
            break;

        case APP_TC_STATE_AT_TARGET:
            /* 先制动吸收残余惯性，再转普通停机。 */
            ctx->tickCount++;
            if (ctx->tickCount <= APP_TC_BRAKE_HOLD_TICKS)
            {
                BSP_Motor_Brake();
            }
            else
            {
                BSP_Motor_Stop();
                ctx->tickCount = APP_TC_BRAKE_HOLD_TICKS + 1U;
            }
            ctx->motionDir = BSP_MOTOR_STOP;
            break;

        case APP_TC_STATE_ERROR:
            BSP_Motor_Stop();
            ctx->motionDir = BSP_MOTOR_STOP;
            break;

        default:
            SetError(ctx, APP_TC_ERR_DIRECTION);
            break;
    }
}

bool APP_TC_IsAtTarget(const AppTcCtx_t *ctx)
{
    return (ctx->state == APP_TC_STATE_AT_TARGET);
}

bool APP_TC_IsError(const AppTcCtx_t *ctx)
{
    return (ctx->state == APP_TC_STATE_ERROR);
}

AppTcPos_t APP_TC_GetArrivedPos(const AppTcCtx_t *ctx)
{
    if (ctx->state == APP_TC_STATE_AT_TARGET)
    {
        return ctx->lastKnownPos;
    }

    return APP_TC_POS_NONE;
}

void APP_TC_Reset(AppTcCtx_t *ctx)
{
    BSP_PositionGpio_ArmStop(BSP_POS_ARM_NONE);
    BSP_Motor_Stop();

    ctx->state          = APP_TC_STATE_IDLE;
    ctx->stateAfterWait = APP_TC_STATE_IDLE;
    ctx->dirAfterWait   = BSP_MOTOR_STOP;
    ctx->motionDir      = BSP_MOTOR_STOP;
    ctx->tickCount      = 0U;
    ctx->errorCode      = APP_TC_ERR_NONE;
}

/* ── 判断 ───────────────────────────────────────────────────────── */

static AppTcPos_t SwToPos(AppSwState_t swState)
{
    switch (swState)
    {
        case APP_SW_4WD_LOCK: return APP_TC_POS_2WD;
        case APP_SW_4WD:      return APP_TC_POS_4WD;
        case APP_SW_2WD:      return APP_TC_POS_4WD_LOCK;
        default:              return APP_TC_POS_NONE;
    }
}

static bool Is4WdPositionActive(const BspPositionInputs_t *inputs)
{
    return (inputs->fourWdLeftActive || inputs->fourWdRightActive);
}

/**
 * @brief 是否已经到达目标。
 *
 * 4WD 有两个合法落点，必须都认：
 *   FORWARD 停在下降沿 -> 磁铁在霍尔B检测区【内】，B 为低
 *   REVERSE 停在上升沿 -> 磁铁在检测区【外】的间隙里，B 为高
 *
 * atFourWd 由 ArriveAt() 置位，覆盖刚定位完的情况；fourWdRightActive
 * 覆盖上电即压在检测区内、以及目标切走又切回的情况。
 *
 * 若只认其中一个，另一个落点会被判成"未到位"，随后被命令走向对方，
 * 到了对方又被命令走回来——两个落点互相驱赶，形成往复。
 */
static bool AlreadyAtTarget(const AppTcCtx_t *ctx,
                            const BspPositionInputs_t *inputs)
{
    switch (ctx->target)
    {
        case APP_TC_POS_4WD_LOCK: return inputs->lockEndActive;
        case APP_TC_POS_2WD:      return inputs->twoWdEndActive;
        case APP_TC_POS_4WD:      return (bool)(ctx->atFourWd ||
                                                inputs->fourWdRightActive);
        default:                  return false;
    }
}

/**
 * @brief 选择运动方向。
 *
 * 两个端点目标的方向由几何确定，永远不会失败。
 *
 * 目标 4WD 时只会在霍尔B 未压住的情况下被调用——压住时 AlreadyAtTarget()
 * 已经返回真。因此这里不需要、也不能有"压住B 就向左走"的规则。
 *
 * 两个 4WD 霍尔都释放时无法只靠电平判断位于B 的哪一侧，用最近压住过的
 * 霍尔加最近运动方向推断：
 *
 *   lastHall  lastMotionDir   所处位置       方向
 *   --------  --------------  -------------  --------
 *   C         FORWARD         C 与 B 之间    FORWARD
 *   C         REVERSE         C 左侧         FORWARD
 *   B         FORWARD         B 右侧         REVERSE
 *   B         REVERSE         C 与 B 之间    FORWARD
 *
 * 即只有"刚向右离开B"时需要往回走，其余一律向右。
 */
static BspMotorDir_t ChooseDirection(const AppTcCtx_t *ctx,
                                     const BspPositionInputs_t *inputs)
{
    switch (ctx->target)
    {
        case APP_TC_POS_4WD_LOCK:
            return BSP_MOTOR_FORWARD;

        case APP_TC_POS_2WD:
            return BSP_MOTOR_REVERSE;

        case APP_TC_POS_4WD:
            if (inputs->fourWdLeftActive) { return BSP_MOTOR_FORWARD; }
            if (inputs->lockEndActive)    { return BSP_MOTOR_REVERSE; }
            if (inputs->twoWdEndActive)   { return BSP_MOTOR_FORWARD; }

            if (ctx->lastKnownPos == APP_TC_POS_4WD_LOCK)
            {
                return BSP_MOTOR_REVERSE;
            }
            if (ctx->lastKnownPos == APP_TC_POS_2WD)
            {
                return BSP_MOTOR_FORWARD;
            }

            if (ctx->lastHall == APP_TC_HALL_B &&
                ctx->lastMotionDir == BSP_MOTOR_FORWARD)
            {
                return BSP_MOTOR_REVERSE;
            }
            return BSP_MOTOR_FORWARD;

        default:
            return BSP_MOTOR_STOP;
    }
}

/* ── 动作 ───────────────────────────────────────────────────────── */

/**
 * @brief 武装 4WD 停车的预置触发。
 *
 * ★ 必须在电机真正开始运动之后调用 ★
 *
 * 掉头请求会先制动再等 200 ms 换向死区，这段时间电机断电但机构仍按原方向
 * 惯性滑行。若在进入死区前就武装，滑行途中穿过霍尔B 会让中断提前命中并
 * 置位触发标志；死区结束进入 SEEK_4WD 时第一次取标志就拿到这个过期值，
 * 立刻误判到位。外部表现为"冲过停车点再回来一点然后停在错误位置"。
 *
 * 不需要区分起步时霍尔B 是否已压住：判定只由边沿中断触发，进区那次下降沿
 * 与 ARM_B_RISING 不匹配会被忽略。
 */
static void ArmForSeek(BspMotorDir_t dir)
{
    BSP_PositionGpio_ArmStop((dir == BSP_MOTOR_FORWARD)
                                 ? BSP_POS_ARM_B_FALLING
                                 : BSP_POS_ARM_B_RISING);
}

/**
 * @brief 下发运动方向。
 *
 * 方向与当前一致时只改状态，电机不中断。只有 FORWARD 与 REVERSE 之间的
 * 掉头才制动并进入换向死区——这是 H 桥防直通的必要措施。
 *
 * 下发后回读实际方向：命令可能被端点联锁或运行锁定静默拒绝，此时立即
 * 报错，而不是让状态机空跑到超时。
 */
static void RequestMotion(AppTcCtx_t *ctx,
                          BspMotorDir_t newDirection,
                          AppTcState_t nextState)
{
    BspMotorDir_t oldDirection = ctx->motionDir;
    bool reversing;

    reversing =
        ((oldDirection == BSP_MOTOR_FORWARD &&
          newDirection == BSP_MOTOR_REVERSE) ||
         (oldDirection == BSP_MOTOR_REVERSE &&
          newDirection == BSP_MOTOR_FORWARD));

    if (reversing)
    {
        //BSP_DbgLog(DBG_TAG_DIRWAIT, (uint16_t)newDirection); /* 调试 */

        /* 死区内机构仍在滑行，必须保证未武装。 */
        BSP_PositionGpio_ArmStop(BSP_POS_ARM_NONE);

        BSP_Motor_Brake();
        ctx->motionDir      = BSP_MOTOR_STOP;
        ctx->dirAfterWait   = newDirection;
        ctx->stateAfterWait = nextState;
        ctx->state          = APP_TC_STATE_DIR_CHANGE_WAIT;
        ctx->tickCount      = 0U;
        return;
    }

    BSP_Motor_SetDir(newDirection);
    //BSP_DbgLog(DBG_TAG_MOTOR_START, (uint16_t)newDirection); /* 调试 */

    ctx->motionDir = BSP_Motor_GetDir();
    if (ctx->motionDir != BSP_MOTOR_STOP)
    {
        ctx->lastMotionDir = ctx->motionDir;
    }

    if (newDirection != BSP_MOTOR_STOP && ctx->motionDir != newDirection)
    {
        SetError(ctx, APP_TC_ERR_DIRECTION);
        return;
    }

    /* 电机已经在动，此刻才武装。 */
    if (nextState == APP_TC_STATE_SEEK_4WD)
    {
        ctx->seekDir = newDirection;
        ArmForSeek(newDirection);
    }

    ctx->state     = nextState;
    ctx->tickCount = 0U;
}

/**
 * @brief 按当前目标和位置规划一次动作。
 *
 * IDLE 每周期调用，档位改变时也立即调用。函数本身不含等待。
 */
static void Replan(AppTcCtx_t *ctx, const BspPositionInputs_t *inputs)
{
    BspMotorDir_t direction;

    if (AlreadyAtTarget(ctx, inputs))
    {
        ArriveAt(ctx, ctx->target);
        return;
    }

    if (!ctx->homingDone)
    {
        RequestMotion(ctx, BSP_MOTOR_REVERSE, APP_TC_STATE_HOMING);
        return;
    }

    direction = ChooseDirection(ctx, inputs);
    if (direction == BSP_MOTOR_STOP)
    {
        SetError(ctx, APP_TC_ERR_DIRECTION);
        return;
    }

    RequestMotion(ctx, direction,
                  (ctx->target == APP_TC_POS_4WD)
                      ? APP_TC_STATE_SEEK_4WD
                      : APP_TC_STATE_MOVING);
}

static void ArriveAt(AppTcCtx_t *ctx, AppTcPos_t position)
{
    //BSP_DbgLog(DBG_TAG_ARRIVE, (uint16_t)position); /* 调试 */

    /* 解除武装，避免之后路过霍尔B 被中断误制动。 */
    BSP_PositionGpio_ArmStop(BSP_POS_ARM_NONE);
    BSP_Motor_Brake();

    ctx->state        = APP_TC_STATE_AT_TARGET;
    ctx->lastKnownPos = position;
    ctx->homingDone   = true;
    ctx->motionDir    = BSP_MOTOR_STOP;
    ctx->tickCount    = 0U;
    ctx->errorCode    = APP_TC_ERR_NONE;

    if (position == APP_TC_POS_4WD)
    {
        ctx->atFourWd = true;
    }
}

static void SetError(AppTcCtx_t *ctx, AppTcErrCode_t errorCode)
{
    //BSP_DbgLog(DBG_TAG_ERROR, (uint16_t)errorCode); /* 调试 */

    BSP_PositionGpio_ArmStop(BSP_POS_ARM_NONE);
    BSP_Motor_Stop();

    ctx->state     = APP_TC_STATE_ERROR;
    ctx->motionDir = BSP_MOTOR_STOP;
    ctx->tickCount = 0U;
    ctx->errorCode = (uint32_t)errorCode;
}

/* ── 状态处理 ───────────────────────────────────────────────────── */

/**
 * @brief 掉头前的换向死区。
 *
 * 前 100 ms 保持制动吸收惯性，之后转普通停机让 H 桥完全断开，
 * 到期再下发新方向。这是唯一会让电机短暂停顿的路径。
 */
static void HandleDirectionWait(AppTcCtx_t *ctx)
{
    ctx->tickCount++;

    if (ctx->tickCount <= APP_TC_BRAKE_HOLD_TICKS)
    {
        BSP_Motor_Brake();
        return;
    }

    if (ctx->tickCount < APP_TC_DIR_CHANGE_WAIT_TICKS)
    {
        BSP_Motor_Stop();
        return;
    }

    BSP_Motor_SetDir(ctx->dirAfterWait);
    //BSP_DbgLog(DBG_TAG_MOTOR_START, (uint16_t)ctx->dirAfterWait); /* 调试 */

    ctx->motionDir = BSP_Motor_GetDir();
    if (ctx->motionDir != BSP_MOTOR_STOP)
    {
        ctx->lastMotionDir = ctx->motionDir;
    }

    if (ctx->dirAfterWait != BSP_MOTOR_STOP &&
        ctx->motionDir != ctx->dirAfterWait)
    {
        SetError(ctx, APP_TC_ERR_DIRECTION);
        return;
    }

    /* 死区结束、电机刚起步，此刻才武装。 */
    if (ctx->stateAfterWait == APP_TC_STATE_SEEK_4WD)
    {
        ctx->seekDir = ctx->dirAfterWait;
        ArmForSeek(ctx->dirAfterWait);
    }

    ctx->state     = ctx->stateAfterWait;
    ctx->tickCount = 0U;
}

/**
 * @brief 位置未知时沿 REVERSE 寻找基准。
 *
 * 途中碰到任一 4WD 霍尔即可确定位置，不必跑到 2WD 机械端点。位置字段已由
 * APP_TC_Update() 开头刷新，这里只需退出归位状态。
 */
static void HandleHoming(AppTcCtx_t *ctx, const BspPositionInputs_t *inputs)
{
    ctx->tickCount++;

    if (ctx->motionDir != BSP_MOTOR_REVERSE)
    {
        SetError(ctx, APP_TC_ERR_DIRECTION);
        return;
    }

    if (Is4WdPositionActive(inputs) || inputs->twoWdEndActive)
    {
        ctx->state = APP_TC_STATE_IDLE;
        Replan(ctx, inputs);
        return;
    }

    if (inputs->lockEndActive)
    {
        SetError(ctx, APP_TC_ERR_DIRECTION);
        return;
    }

    if (ctx->tickCount >= APP_TC_HOMING_TIMEOUT_TICKS)
    {
        SetError(ctx, APP_TC_ERR_HOMING_FAILED);
    }
}

/** @brief 向 2WD 或 4WD_LOCK 端点运动，按电平判定到位。 */
static void HandleMoving(AppTcCtx_t *ctx, const BspPositionInputs_t *inputs)
{
    ctx->tickCount++;

    if (ctx->tickCount >= APP_TC_TIMEOUT_TICKS)
    {
        SetError(ctx, APP_TC_ERR_TIMEOUT_MOVE);
        return;
    }

    if ((ctx->target == APP_TC_POS_4WD_LOCK &&
         ctx->motionDir != BSP_MOTOR_FORWARD) ||
        (ctx->target == APP_TC_POS_2WD &&
         ctx->motionDir != BSP_MOTOR_REVERSE))
    {
        SetError(ctx, APP_TC_ERR_DIRECTION);
        return;
    }

    if (ctx->target == APP_TC_POS_4WD_LOCK && inputs->lockEndActive)
    {
        ArriveAt(ctx, APP_TC_POS_4WD_LOCK);
    }
    else if (ctx->target == APP_TC_POS_2WD && inputs->twoWdEndActive)
    {
        ArriveAt(ctx, APP_TC_POS_2WD);
    }
}

/**
 * @brief 向 4WD 运动，等 PTA1 中断触发。
 *
 * 制动已在中断里完成，这里只取标志并迁移状态。冲到运动方向远端的机械
 * 端点说明越界；起点侧端点在刚起步时仍然有效，必须忽略。
 */
static void HandleSeek4WD(AppTcCtx_t *ctx, const BspPositionInputs_t *inputs)
{
    ctx->tickCount++;

    /* 命中时中断已经制动，必须最先取标志。 */
    if (BSP_PositionGpio_ConsumeStopTrigger())
    {
        ArriveAt(ctx, APP_TC_POS_4WD);
        return;
    }

    if (ctx->tickCount >= APP_TC_SEEK_TIMEOUT_TICKS)
    {
        SetError(ctx, APP_TC_ERR_TIMEOUT_SEEK);
        return;
    }

    if ((ctx->seekDir == BSP_MOTOR_FORWARD && inputs->lockEndActive) ||
        (ctx->seekDir == BSP_MOTOR_REVERSE && inputs->twoWdEndActive))
    {
        SetError(ctx, APP_TC_ERR_DIRECTION);
    }
}