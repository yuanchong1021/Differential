/**
 * @file    task_sched.h
 * @brief   协作式任务调度器接口
 * @layer   TASK（任务调度层）
 *
 * 调度模型：
 *   - 基于 LPTMR0 比较中断产生 1 ms 软件时基；
 *   - 主循环调用 SCHED_Run()，依次检查每个任务的 "到期" 条件，
 *     满足则执行一次任务函数（协作式，任务自身不应无限阻塞）；
 *   - 任务均在主循环上下文执行，无抢占，无需互斥锁。
 *
 * 任务优先级（执行顺序）：按注册先后顺序，序号小者先运行。
 * 当多个任务同一 tick 到期时，按序依次执行（同周期内不重入）。
 *
 * 使用示例：
 * @code
 *   SCHED_Init();
 *   SCHED_RegisterTask("Position", Task_Position_Run, 1); // 每 1 ms
 *   SCHED_RegisterTask("Switch", Task_Switch_Run, 10);  // 每 10 ms
 *   SCHED_RegisterTask("Ctrl",   Task_Ctrl_Run,   10);  // 每 10 ms
 *   while (1) { SCHED_Run(); }
 * @endcode
 */

#ifndef TASK_SCHED_H
#define TASK_SCHED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 调度器配置 ───────────────────────────────────────────────────── */
#define SCHED_MAX_TASKS    (8U)    /**< 最大任务数量                   */
#define SCHED_TICK_FREQ_HZ (1000U) /**< 调度时基频率：1000 Hz = 1 ms */
#define SCHED_CLK_HZ       (48000000U) /**< 系统时钟：FIRC 48 MHz     */

/* ── 任务函数类型 ─────────────────────────────────────────────────── */
typedef void (*SchedTaskFunc_t)(void);

/* ── 任务描述符（调用者无需手动填充，通过 RegisterTask 注册） ─────── */
typedef struct {
    SchedTaskFunc_t func;          /**< 任务函数指针                  */
    uint32_t        periodMs;      /**< 执行周期（毫秒），0=每 tick   */
    uint32_t        nextRunTick;   /**< 下次运行的绝对 tick 值        */
    bool            enabled;       /**< 任务使能标志                  */
    const char     *name;          /**< 任务名称（调试用）            */
} SchedTask_t;

/* ── 系统 tick 计数（1 ms/tick，由 lpTMR0_IRQHandler 维护） ─────── */
extern volatile uint32_t g_sysTick_ms;

/* ── API ──────────────────────────────────────────────────────────── */

/**
 * @brief 初始化调度器任务表，并使能/启动 LPTMR0 时间基准
 * @note  须在 Board_Init() 之后调用，因为 LPTMR0 已在 Board_Init 中配置
 */
void SCHED_Init(void);

/**
 * @brief 注册一个周期任务
 * @param name      任务名（调试标识，可为 NULL）
 * @param func      任务函数
 * @param periodMs  执行周期（毫秒）
 * @return true  注册成功
 * @return false 任务表已满（超过 SCHED_MAX_TASKS）
 */
bool SCHED_RegisterTask(const char     *name,
                        SchedTaskFunc_t func,
                        uint32_t        periodMs);

/**
 * @brief 调度器主函数（在 main 无限循环中调用）
 *
 * 遍历任务表，对到期任务执行一次 func()。
 * 本函数返回前完成所有到期任务的一轮执行。
 */
void SCHED_Run(void);

/**
 * @brief 使能/禁用某个任务（按函数指针查找）
 */
void SCHED_EnableTask(SchedTaskFunc_t func, bool enable);

/**
 * @brief 获取当前系统 tick（毫秒）
 */
static inline uint32_t SCHED_GetTick(void)
{
    return g_sysTick_ms;
}

#ifdef __cplusplus
}
#endif

#endif /* TASK_SCHED_H */
