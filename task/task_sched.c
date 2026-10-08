/**
 * @file    task_sched.c
 * @brief   协作式任务调度器实现
 *
 * 时间基准由 LPTMR0 提供，配置参数在 Board_Init() 中由 LPTMR_Config
 * 初始化。本文件使能 lpTMR0_IRQn 并启动计数器；每次比较中断清标志后，
 * 将 g_sysTick_ms 增加 1。工程配置必须保证一次中断对应 1 ms。
 *
 * 所有任务在 main() 主循环中协作式执行，中断只维护毫秒计数，不在中断
 * 中调用业务任务。
 */

#include "task_sched.h"
#include "YTM32B1LE1.h"   /* 芯片寄存器及中断定义 */
#include "lptmr_driver.h"
#include "interrupt_manager.h"
#include <string.h>

/* ── 全局 tick 计数 ───────────────────────────────────────────────── */
volatile uint32_t g_sysTick_ms = 0U;

/* ── 任务表 ───────────────────────────────────────────────────────── */
static SchedTask_t s_tasks[SCHED_MAX_TASKS];
static uint32_t    s_taskCount = 0U;

/* ── LPTMR0 1 ms 中断服务程序：只清标志并累加时间基准 ───────────── */
void lpTMR0_IRQHandler(void)
{
    lpTMR_DRV_ClearCompareFlag(0u);
    g_sysTick_ms++;
}

/* ── 公共 API ─────────────────────────────────────────────────────── */

void SCHED_Init(void)
{
    memset(s_tasks, 0, sizeof(s_tasks));
    s_taskCount   = 0U;
    g_sysTick_ms  = 0U;

    INT_SYS_EnableIRQ(lpTMR0_IRQn);
    lpTMR_DRV_StartCounter(0u);
}

bool SCHED_RegisterTask(const char     *name,
                        SchedTaskFunc_t func,
                        uint32_t        periodMs)
{
    if (s_taskCount >= SCHED_MAX_TASKS || func == (SchedTaskFunc_t)0) 
    {
        return false;
    }

    SchedTask_t *t  = &s_tasks[s_taskCount];
    t->func         = func;
    t->periodMs     = periodMs;
    t->nextRunTick  = g_sysTick_ms + periodMs; /* 延迟一个周期后首次执行 */
    t->enabled      = true;
    t->name         = name;
    s_taskCount++;
    return true;
}

void SCHED_Run(void)
{
    uint32_t   now = g_sysTick_ms;  /* 快照一次，避免 ISR 期间 tick 跳变 */
    uint32_t   i;

    for (i = 0U; 
         
         i < s_taskCount; i++)
    {
        SchedTask_t *t = &s_tasks[i];
        if (!t->enabled || t->func == (SchedTaskFunc_t)0)
        {
            continue;
        }

        /* 判断是否到期（使用无符号差值，可处理 tick 绕回） */
        if ((now - t->nextRunTick) < 0x80000000U)
        {
            t->func();                        /* 执行任务               */
            t->nextRunTick = now + t->periodMs; /* 安排下次运行时间     */
        }
    }
}

void SCHED_EnableTask(SchedTaskFunc_t func, bool enable)
{
    uint32_t i;
    for (i = 0U; i < s_taskCount; i++)
    {
        if (s_tasks[i].func == func)
        {
            s_tasks[i].enabled = enable;
            break;
        }
    }
}
