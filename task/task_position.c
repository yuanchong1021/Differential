/**
 * @file    task_position.c
 * @brief   位置 GPIO 1 ms 采样、上电稳定确认及事件转发
 *
 * 启动稳定算法：只有四路输入位掩码连续 20 次（20 ms）完全相同，才允许
 * 控制状态机运行。任意一路发生变化都会重新开始计数。稳定期间始终调用
 * BSP_Motor_Stop()，从软件流程上消除“上电先动一下再停”的窗口。
 */

#include "task_position.h"
#include "bsp_motor.h"

/* 上一次采样到的四路有效位，用于判断连续样本是否一致。 */
static uint8_t  s_lastMask    = BSP_POS_EVENT_NONE;

/* 连续一致样本数量；达到 TASK_POSITION_STABLE_COUNT 后置 ready。 */
static uint16_t s_stableCount = 0U;

/* 启动输入是否已经通过稳定确认。 */
static bool     s_ready       = false;

/**
 * @brief 初始化位置 GPIO BSP，并开始上电稳定确认。
 *
 * 初始化后不会立即把 s_ready 置为 true，即使第一次采样看起来有效，也必须
 * 继续观察完整稳定窗口。这样可过滤上电瞬态和初始化阶段产生的伪边沿。
 */
void Task_Position_Init(void)
{
    BSP_PositionGpio_Init();
    BSP_PositionGpio_Sample();

    s_lastMask    = BSP_PositionGpio_GetActiveMask();
    s_stableCount = 0U;
    s_ready       = false;

    /* 稳定确认完成前不允许状态机驱动电机。 */
    BSP_Motor_Stop();
    BSP_PositionGpio_ClearFallingEvents();
}

/** @brief 1 ms 周期函数：刷新电平并推进启动稳定计数。 */
void Task_Position_Run(void)
{
    uint8_t currentMask;

    BSP_PositionGpio_Sample();

    if (!BSP_PositionGpio_IsInitialized())
    {
        s_ready = false;
        BSP_Motor_Stop();
        return;
    }

    currentMask = BSP_PositionGpio_GetActiveMask();

    if (s_ready)
    {
        return;
    }

    /*
     * 上电先等待四路电平连续稳定，避免引脚上拉尚未建立或输入抖动时
     * 状态机提前启动，造成“上电微动并逐次偏离”。
     */
    BSP_Motor_Stop();

    if (currentMask == s_lastMask)
    {
        if (s_stableCount < TASK_POSITION_STABLE_COUNT)
        {
            s_stableCount++;
        }
    }
    else
    {
        s_lastMask    = currentMask;
        s_stableCount = 1U;
    }

    if (s_stableCount >= TASK_POSITION_STABLE_COUNT)
    {
        s_ready = true;

        /* 启动稳定阶段产生的边沿不属于电机运动事件。 */
        BSP_PositionGpio_ClearFallingEvents();
    }
}

const BspPositionInputs_t *Task_Position_GetInputs(void)
{
    return BSP_PositionGpio_GetInputs();
}

uint8_t Task_Position_ConsumeFallingEvents(void)
{
    return BSP_PositionGpio_ConsumeFallingEvents();
}

bool Task_Position_IsReady(void)
{
    return s_ready;
}
