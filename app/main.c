/* USER CODE BEGIN Header */
/**
 * @file    main.c
 * @brief   分动器电机控制主程序入口
 *
 * 软件分层：
 *   TASK：task_sched / task_position / task_switch / task_ctrl
 *   APP ：app_tc
 *   BSP ：bsp_motor / bsp_position_gpio / bsp_switch / bsp_indicator
 *
 * 周期任务：
 *   Task_Position_Run  1 ms：采样位置 GPIO 并完成上电稳定确认
 *   Task_Switch_Run   10 ms：读取目标开关并消抖
 *   Task_Ctrl_Run      2 ms：运行分动器状态机
 *
 * 四路位置输入的下降沿由 GPIOA/B/D_IRQHandler() 分端口异步锁存。
 *
 * 上电安全初始化顺序：
 *   1. 时钟和全部 pin_mux 初始化；
 *   2. MPWM 初始化；
 *   3. BSP_Motor_Init() 立即建立 LOW/LOW 普通停机；
 *   4. Task_Position_Init() 清位置中断标志并读取初始电平；
 *   5. 输入连续稳定 20 ms 后，控制任务才允许状态机驱动电机。
 */
/* USER CODE END Header */
#include "sdk_project_config.h"

/* USER CODE BEGIN Includes */
#include "task_sched.h"
#include "task_position.h"
#include "task_switch.h"
#include "task_ctrl.h"
#include "bsp_motor.h"
/* USER CODE END Includes */

static void Board_Init(void);
volatile uint32_t g_dbgRstSrc;   /* 文件作用域全局，不要放函数里 */

int main(void)
{

    g_dbgRstSrc = RCU->RSSR;
    RCU->RSSR   = g_dbgRstSrc;
    /* 完成所有硬件初始化，并确保电机输出首先处于普通停机。 */
    Board_Init();
/* main() 里 Board_Init() 之后 */

RCU->RPFR = RCU_RPFR_FLTVAL(31U)   /* 滤波宽度，先取较大值 */
          | RCU_RPFR_FLTCLKSEL(0U) /* 参考手册确认时钟源 */
          | RCU_RPFR_FLTEN_MASK;   /* 使能滤波 */
          
    /*
     * 任务初始化顺序不能随意调整：位置模块先建立端点联锁，开关模块读取
     * 初始目标，最后控制模块创建状态机上下文。
     */
    Task_Position_Init();
    Task_Switch_Init();
    Task_Ctrl_Init();

    /* 同一 tick 到期时按 Position -> Switch -> Ctrl 顺序执行。 */
    SCHED_Init();
    SCHED_RegisterTask("Position",
                       Task_Position_Run,
                       TASK_POSITION_PERIOD_MS);
    SCHED_RegisterTask("Switch",
                       Task_Switch_Run,
                       TASK_SW_PERIOD_MS);
    SCHED_RegisterTask("Ctrl",
                       Task_Ctrl_Run,
                       TASK_CTRL_PERIOD_MS);

    while (1)
    {
        /* 协作式调度：所有周期任务均在主循环上下文执行。 */
        SCHED_Run();
    }
}

static void Board_Init(void)
{
    CLOCK_SYS_Init(g_clockManConfigsArr,
                   CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr,
                   CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(CLOCK_MANAGER_ACTIVE_INDEX,
                                  CLOCK_MANAGER_POLICY_AGREEMENT);

    /*
     * 全部引脚只在这里初始化一次。g_pin_mux_InitConfigArr0 必须包含：
     *   PTA0：GPIO 输入、下降沿中断、4WD_LOCK 端点
     *   PTA1：GPIO 输入、下降沿中断、4WD 右位置
     *   PTD3：GPIO 输入、下降沿中断、4WD 左位置
     *   PTB2：GPIO 输入、下降沿中断、2WD 端点
     */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);

    MPWM_DRV_Init(0, &MPWM_State0);
    MPWM_DRV_InitPwm(0, &MPWM_PWM_Config0);

    /* PWM 建立后立即输出 LOW/LOW 普通停机，消除初始化微动窗口。 */
    BSP_Motor_Init();

    lpTMR_DRV_Init(0, &LPTMR_Config, false);
}
