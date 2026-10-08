/**
 * @file    task_position.h
 * @brief   位置 GPIO 电平采样、上电稳定确认和下降沿事件接口
 * @layer   TASK
 *
 * 该任务每 1 ms 采样一次四路位置 GPIO，并在系统启动时执行 20 ms 稳定确认。
 * 稳定确认完成前，Task_Position_IsReady() 始终返回 false，控制任务必须保持
 * 电机 STOP。这样可以避免上电过程中外部上拉尚未稳定、引脚抖动或旧中断
 * 标志造成一次错误驱动。
 *
 * 下降沿中断与周期采样互为补充：
 *   - 中断负责“不漏掉曾经出现过的短脉冲”；
 *   - 周期采样负责“知道输入当前是否已经释放”。
 */

#ifndef TASK_POSITION_H
#define TASK_POSITION_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_position_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TASK_POSITION_PERIOD_MS          (1U)
#define TASK_POSITION_STARTUP_STABLE_MS  (10U)
#define TASK_POSITION_STABLE_COUNT       \
    (TASK_POSITION_STARTUP_STABLE_MS / TASK_POSITION_PERIOD_MS)

void Task_Position_Init(void);
void Task_Position_Run(void);

/** 获取最近一次四路位置输入快照。 */
const BspPositionInputs_t *Task_Position_GetInputs(void);

/** 取出并清除自上次调用以来锁存的 GPIO 下降沿事件。 */
uint8_t Task_Position_ConsumeFallingEvents(void);

/**
 * 四路输入连续稳定 TASK_POSITION_STARTUP_STABLE_MS 后返回 true。
 * ready 之前控制任务必须保持电机普通停机。
 */
bool Task_Position_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_POSITION_H */
