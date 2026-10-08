/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-24 16:57:18
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-09-03 11:16:57
 * @FilePath: \Differential\task\task_ctrl.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file    task_ctrl.h
 * @brief   分动器控制任务接口
 * @layer   TASK
 *
 * 职责：
 *   - 从 task_position、task_switch 获取最新输入；
 *   - 驱动 app_tc 状态机；
 *   - 维护电机运行保护和位置指示输出。
 *
 * task_ctrl 必须在 task_position 和 task_switch 之后运行。
 *
 * 单个控制周期的数据处理顺序：
 *   1. 确认位置输入已经通过上电稳定检查；
 *   2. 复制当前 GPIO 电平快照；
 *   3. 取出 ISR 锁存的下降沿，并在本周期临时合并进快照；
 *   4. 读取经消抖的目标开关；
 *   5. 调用 APP_TC_Update() 推进状态机；
 *   6. 更新电机连续运行保护和位置指示输出。
 */

#ifndef TASK_CTRL_H
#define TASK_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 控制任务周期。2 ms 可降低输入事件到应用停机确认的延迟。 */
#define TASK_CTRL_PERIOD_MS  (1U)

void Task_Ctrl_Init(void);
void Task_Ctrl_Run(void);
bool Task_Ctrl_IsAtTarget(void);
bool Task_Ctrl_IsError(void);
void Task_Ctrl_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_CTRL_H */
