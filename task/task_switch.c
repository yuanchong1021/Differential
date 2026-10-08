/**
 * @file    task_switch.c
 * @brief   档位开关采集与消抖
 */

#include "task_switch.h"
#include "bsp_switch.h"

static AppSwState_t s_stableState    = APP_SW_4WD; /* 消抖后的输出   */
static AppSwState_t s_candidateState = APP_SW_4WD; /* 待确认的读数   */
static uint16_t     s_debounceCount  = 0U;         /* 连续相同次数   */

/**
 * @brief 把两路开关电平译码为档位。
 *
 * SWA 是 4WD_LOCK 请求，优先级最高；只有 SWA 断开时才看 SWB。
 * 两路都断开表示 4WD。
 */
static AppSwState_t ReadRaw(void)
{
    BspSwLevel_t swa = BSP_Switch_ReadSWA();
    BspSwLevel_t swb = BSP_Switch_ReadSWB();

    if (swa == BSP_SW_LEVEL_LOW) { return APP_SW_4WD_LOCK; }
    if (swb == BSP_SW_LEVEL_LOW) { return APP_SW_2WD;      }
    return APP_SW_4WD;
}

void Task_Switch_Init(void)
{
    BSP_Switch_Init();

    /* 预读初始状态，直接视为已稳定。 */
    s_stableState    = ReadRaw();
    s_candidateState = s_stableState;
    s_debounceCount  = TASK_SW_DEBOUNCE_CNT;
}

void Task_Switch_Run(void)
{
    AppSwState_t current = ReadRaw();

    if (current != s_candidateState)
    {
        s_candidateState = current;
        s_debounceCount  = 1U;
        return;
    }

    if (s_debounceCount < TASK_SW_DEBOUNCE_CNT)
    {
        s_debounceCount++;
    }

    if (s_debounceCount >= TASK_SW_DEBOUNCE_CNT)
    {
        s_stableState = s_candidateState;
    }
}

AppSwState_t Task_Switch_GetState(void)
{
    return s_stableState;
}