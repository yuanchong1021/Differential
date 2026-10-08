/**
 * @file    bsp_switch.c
 * @brief   SWA / SWB 开关量采集 BSP 实现
 *
 * 外部上拉电阻已由硬件提供，本层仅做 GPIO 原始读取，
 * 消抖逻辑由上层 task_switch 负责。
 */

#include "bsp_switch.h"

void BSP_Switch_Init(void)
{
    /*
     * pin_mux.c 已将 PTE_4 / PTE_5 配置为 GPIO 输入 + 无内部上拉。
     * 外部上拉电阻已在 PCB 上焊接，无需额外配置。
     * 若将来硬件更改去掉外部上拉，可在此处启用：
     *   PINS_DRV_SetPullSel(PCTRLE, BSP_SWA_PIN, PCTRL_INTERNAL_PULL_UP_ENABLED);
     *   PINS_DRV_SetPullSel(PCTRLE, BSP_SWB_PIN, PCTRL_INTERNAL_PULL_UP_ENABLED);
     */
}

/** @brief 单次读取 SWA，不做消抖；低电平表示开关闭合到地。 */
BspSwLevel_t BSP_Switch_ReadSWA(void)
{
    return (PINS_DRV_ReadPin(BSP_SW_GPIO, BSP_SWA_PIN) == 0U)
           ? BSP_SW_LEVEL_LOW : BSP_SW_LEVEL_HIZ;
}

/** @brief 单次读取 SWB，不做消抖；低电平表示开关闭合到地。 */
BspSwLevel_t BSP_Switch_ReadSWB(void)
{
    return (PINS_DRV_ReadPin(BSP_SW_GPIO, BSP_SWB_PIN) == 0U)
           ? BSP_SW_LEVEL_LOW : BSP_SW_LEVEL_HIZ;
}
