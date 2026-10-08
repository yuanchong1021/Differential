/**
 * @file    task_switch.h
 * @brief   档位开关采集接口
 * @layer   TASK
 *
 * 引脚：SWA -> PTE_4，SWB -> PTE_5，接地有效。
 *
 * 译码（SWA 优先级最高）：
 *   SWA 接地              -> APP_SW_4WD_LOCK
 *   SWA 断开，SWB 接地    -> APP_SW_2WD
 *   两路均断开            -> APP_SW_4WD
 *
 * 消抖：读数连续 TASK_SW_DEBOUNCE_CNT 次相同才更新稳定输出，任意一次
 * 不同即重新计数。三个档位使用同一个门限。
 */

#ifndef TASK_SWITCH_H
#define TASK_SWITCH_H

#include "bsp_switch.h"
#include "app_tc.h"        /* 使用 AppSwState_t */

#ifdef __cplusplus
extern "C" {
#endif

/** 采集周期（毫秒） */
#define TASK_SW_PERIOD_MS      (10U)

/** 消抖次数：10 ms x 5 = 50 ms */
#define TASK_SW_DEBOUNCE_CNT   (5U)

/** 初始化：调用 BSP_Switch_Init，预读初始状态 */
void Task_Switch_Init(void);

/** 任务函数，每 TASK_SW_PERIOD_MS 执行一次 */
void Task_Switch_Run(void);

/** 获取消抖后的稳定档位 */
AppSwState_t Task_Switch_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SWITCH_H */