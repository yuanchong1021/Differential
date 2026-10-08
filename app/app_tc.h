/**
 * @file    app_tc.h
 * @brief   分动器档位控制状态机
 * @layer   APP
 *
 * 物理位置顺序：
 *
 *   [2WD端点] ---- [霍尔C] ---- 间隙 ---- [霍尔B] ---- [4WD_LOCK端点]
 *      PTB2         PTD3                   PTA1            PTA0
 *
 *   REVERSE  <-------------------------------------------->  FORWARD
 *
 * ── 停车判据 ────────────────────────────────────────────────────
 *
 *   2WD      ：PTB2 电平有效
 *   4WD_LOCK ：PTA0 电平有效
 *   4WD      ：PTA1 的边沿中断
 *              FORWARD 从左侧接近 -> 下降沿（进入检测区）即停
 *              REVERSE 从右侧接近 -> 上升沿（走出检测区）即停
 *
 * 4WD 的两个落点分别在霍尔B检测区的内侧和外侧，都是合法位置。
 * 判定由 BSP 层的预置触发在中断里完成，本层只做状态迁移。
 *
 * ── 设计约束 ────────────────────────────────────────────────────
 *   1. 电机全程单一速度，不做减速逼近；
 *   2. 只按档位开关要求的方向运动，不做自动往复或纠偏；
 *   3. 档位改变时立即重新规划，方向相同则继续走，仅掉头插入换向死区。
 */

#ifndef APP_TC_H
#define APP_TC_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_motor.h"
#include "bsp_position_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 本状态机由 2 ms 周期的控制任务驱动。 */
#define APP_TC_TIMEOUT_TICKS          (3000U) /* 6 s：端点目标运动超时 */
#define APP_TC_HOMING_TIMEOUT_TICKS   (2000U) /* 4 s：归位超时         */
#define APP_TC_SEEK_TIMEOUT_TICKS     (3000U) /* 6 s：4WD 定位超时     */
#define APP_TC_DIR_CHANGE_WAIT_TICKS  (100U)  /* 200 ms：换向死区      */
#define APP_TC_BRAKE_HOLD_TICKS       (50U)   /* 100 ms：制动保持      */

typedef enum
{
    APP_SW_4WD_LOCK = 0,
    APP_SW_4WD      = 1,
    APP_SW_2WD      = 2,
} AppSwState_t;

typedef enum
{
    APP_TC_POS_NONE     = 0,
    APP_TC_POS_2WD      = 1,
    APP_TC_POS_4WD      = 2,
    APP_TC_POS_4WD_LOCK = 3,
} AppTcPos_t;

/** 最近压住过的 4WD 霍尔，用于判断当前位于霍尔B的哪一侧。 */
typedef enum
{
    APP_TC_HALL_NONE = 0,
    APP_TC_HALL_C    = 1, /**< PTD3，靠 2WD 侧  */
    APP_TC_HALL_B    = 2, /**< PTA1，靠 LOCK 侧 */
} AppTcHall_t;

typedef enum
{
    APP_TC_STATE_IDLE            = 0, /**< 按当前目标规划运动        */
    APP_TC_STATE_HOMING          = 1, /**< 位置未知，反转寻找基准    */
    /* 数值 2 保留。 */
    APP_TC_STATE_DIR_CHANGE_WAIT = 3, /**< 掉头前的 H 桥换向死区     */
    APP_TC_STATE_MOVING          = 4, /**< 向 2WD 或 LOCK 端点运动   */
    APP_TC_STATE_SEEK_4WD        = 5, /**< 向 4WD 运动，等中断触发   */
    APP_TC_STATE_AT_TARGET       = 6, /**< 已到位，制动后保持停机    */
    APP_TC_STATE_ERROR           = 7, /**< 超时或位置异常，保持停机  */
} AppTcState_t;

typedef enum
{
    APP_TC_ERR_NONE          = 0,
    APP_TC_ERR_TIMEOUT_MOVE  = 1,
    APP_TC_ERR_TIMEOUT_SEEK  = 2,
    APP_TC_ERR_HOMING_FAILED = 3,
    APP_TC_ERR_DIRECTION     = 4,
} AppTcErrCode_t;

/** 状态机运行上下文。所有字段只由 APP_TC_xxx 接口维护。 */
typedef struct
{
    AppTcState_t  state;          /**< 当前状态                        */
    AppTcState_t  stateAfterWait; /**< 换向死区结束后进入的状态        */
    BspMotorDir_t dirAfterWait;   /**< 换向死区结束后下发的方向        */
    BspMotorDir_t motionDir;      /**< 当前实际运动方向，回读自 BSP    */
    BspMotorDir_t lastMotionDir;  /**< 最近一次非停机的运动方向        */
    BspMotorDir_t seekDir;        /**< 本次 4WD 定位的运动方向         */
    AppTcPos_t    target;         /**< 档位开关要求的目标位置          */
    AppTcPos_t    lastKnownPos;   /**< 最近一次可信确认的位置          */
    AppTcHall_t   lastHall;       /**< 最近压住过的 4WD 霍尔           */
    bool          homingDone;     /**< 是否已建立绝对位置              */
    bool          atFourWd;       /**< 是否已完成一次 4WD 定位         */
    uint32_t      tickCount;      /**< 当前状态持续的 2 ms 计数        */
    uint32_t      errorCode;      /**< AppTcErrCode_t                  */
} AppTcCtx_t;

void APP_TC_Init(AppTcCtx_t *ctx);

/** 启动前按稳定电平预加载当前位置。 */
void APP_TC_PreloadPos(AppTcCtx_t *ctx,
                       const BspPositionInputs_t *positionInputs);

/** 每 2 ms 调用一次。 */
void APP_TC_Update(AppTcCtx_t *ctx,
                   const BspPositionInputs_t *positionInputs,
                   AppSwState_t swState);

bool APP_TC_IsAtTarget(const AppTcCtx_t *ctx);
bool APP_TC_IsError(const AppTcCtx_t *ctx);
AppTcPos_t APP_TC_GetArrivedPos(const AppTcCtx_t *ctx);
void APP_TC_Reset(AppTcCtx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* APP_TC_H */