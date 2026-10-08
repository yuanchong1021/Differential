/**
 * @file    bsp_motor.h
 * @brief   电机驱动 BSP 接口（单一运行速度）
 * @layer   BSP
 *
 * 硬件：MPWM0 CH0（PWMA / PTA3）和 CH1（PWMB / PTA2）。
 *
 * 物理方向：
 *   FORWARD：向 4WD_LOCK 端点运动
 *   REVERSE：向 2WD 端点运动
 *
 * 运行速度固定为 BSP_MOTOR_DUTY_RUN，不做减速。
 *
 * 相对原工程唯一的功能增加是 BSP_Motor_Brake()，用于到位和换向前吸收
 * 残余惯性。它不改变运行速度，只影响停止过程。
 *
 * 保护层级：
 *   1. GPIO 事件建立端点方向联锁，不直接操作 PWM；
 *   2. Task_Ctrl_Run() 在任务上下文执行停机；
 *   3. SetDir() 即使再次收到危险方向命令，也会被端点联锁拒绝；
 *   4. 连续运行超过 15 s 后软件锁定。
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "mpwm_common.h"
#include "mpwm_pwm_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MPWM 外设实例及两个方向控制通道。 */
#define BSP_MOTOR_MPWM_INST   (0U)
#define BSP_MOTOR_CH_PWMA     (0U)  /**< PWMA：PTA3 / MPWM0_CH0 */
#define BSP_MOTOR_CH_PWMB     (1U)  /**< PWMB：PTA2 / MPWM0_CH1 */
#define BSP_MOTOR_CH_MASK     \
    ((1U << BSP_MOTOR_CH_PWMA) | (1U << BSP_MOTOR_CH_PWMB))

/* MPWM_MAX_DUTY_CYCLE = 0x8000 表示 100%。 */
#define BSP_MOTOR_DUTY_LOW    (0x0000U)
#define BSP_MOTOR_DUTY_FULL   (0x8000U)

/** 运行占空比：0x6000 = 75%，与原工程一致，全程唯一速度。 */
#define BSP_MOTOR_DUTY_RUN    (0x6000U)

/* 普通停机输出（两路占空比均为 0）。 */
#ifndef BSP_MOTOR_STOP_DUTY_A
#define BSP_MOTOR_STOP_DUTY_A BSP_MOTOR_DUTY_LOW
#endif
#ifndef BSP_MOTOR_STOP_DUTY_B
#define BSP_MOTOR_STOP_DUTY_B BSP_MOTOR_DUTY_LOW
#endif

/**
 * 主动制动使能开关。
 *
 * ★ 打开之前必须先确认 H 桥真值表 ★
 * 置 1 时 BSP_Motor_Brake() 输出 PWMA=PWMB=HIGH。对多数 IN1/IN2 型 H 桥，
 * 该组合表示两个上桥臂（或下桥臂）同时导通，绕组被短接产生制动。但也有
 * 芯片把它定义为待机甚至直通。查清楚之前保持 0，此时 Brake() 退化为普通
 * 停机（滑行），停车散布会明显变大。
 */
#ifndef BSP_MOTOR_USE_ACTIVE_BRAKE
#define BSP_MOTOR_USE_ACTIVE_BRAKE (1U)
#endif

/*
 * BSP_Motor_Tick() 由 2 ms 控制任务调用。
 * 7500 × 2 ms = 15 s，作为应用状态机之外的连续运行硬保护。
 */
#define BSP_MOTOR_MAX_RUN_TICKS  (7500U)

/** 电机输出方向。数值只用于软件状态，不直接等同于硬件电平。 */
typedef enum
{
    BSP_MOTOR_STOP    = 0, /**< 两路占空比为 0，普通停机   */
    BSP_MOTOR_FORWARD = 1, /**< 朝 4WD_LOCK 端点方向运动   */
    BSP_MOTOR_REVERSE = 2, /**< 朝 2WD 端点方向运动        */
} BspMotorDir_t;

/** PWM 初始化完成后调用，启动计数器并立即建立普通停机输出。 */
void BSP_Motor_Init(void);

/** 设置运动方向，速度固定为 BSP_MOTOR_DUTY_RUN。 */
void BSP_Motor_SetDir(BspMotorDir_t dir);

/**
 * @brief 主动制动。
 * @note  BSP_MOTOR_USE_ACTIVE_BRAKE 为 0 时等价于 BSP_Motor_Stop()。
 *        制动期间 GetDir() 返回 BSP_MOTOR_STOP。制动有电流，调用方必须
 *        在有限时间内切回 Stop()。
 */
void BSP_Motor_Brake(void);

/** 获取当前输出方向。制动和停机均返回 BSP_MOTOR_STOP。 */
BspMotorDir_t BSP_Motor_GetDir(void);

/**
 * 更新两个端点输入状态。
 * @param atLockEndstop true：4WD_LOCK 端点输入当前有效
 * @param at2WdEndstop  true：2WD 端点输入当前有效
 */
void BSP_Motor_UpdateEndstopInputs(bool atLockEndstop, bool at2WdEndstop);

/** 普通停机：立即输出 LOW/LOW。 */
void BSP_Motor_Stop(void);

/** 连续运行时间保护；由控制任务每 2 ms 调用一次。 */
void BSP_Motor_Tick(void);

/** 查询连续运行超时锁定状态。 */
bool BSP_Motor_IsLocked(void);

/** 清除连续运行超时锁定。 */
void BSP_Motor_ClearLock(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */