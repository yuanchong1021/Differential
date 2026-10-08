/**
 * @file    bsp_switch.h
 * @brief   SWA / SWB 开关量采集 BSP 接口
 * @layer   BSP（板级支持层）
 *
 * 硬件引脚：
 *   SWA → PTE_4（GPIOE pin4）
 *   SWB → PTE_5（GPIOE pin5）
 *
 * 外部上拉电阻已存在（硬件已焊接），MCU 引脚保持 pin_mux.c 中的
 * "无内部上下拉（PCTRL_INTERNAL_PULL_NOT_ENABLED）"配置即可。
 *
 * 电气逻辑：
 *   引脚浮空 / 开关断开（Hi-Z） → 外部上拉拉高 → GPIO 读值 = HIGH = 1
 *   开关闭合到 GND（LOW）       → 覆盖外部上拉 → GPIO 读值 = LOW  = 0
 *
 * 消抖：在任务层（task_switch）实现，BSP 仅提供原始单次读取。
 */

#ifndef BSP_SWITCH_H
#define BSP_SWITCH_H

#include <stdint.h>
#include <stdbool.h>
#include "pins_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 引脚定义 ─────────────────────────────────────────────────────── */
#define BSP_SW_GPIO       GPIOE     /**< SWA/SWB 共用 GPIOE   */
#define BSP_SWA_PIN       (4U)      /**< SWA: PTE_4            */
#define BSP_SWB_PIN       (5U)      /**< SWB: PTE_5            */

/* ── 原始电平枚举 ─────────────────────────────────────────────────── */
typedef enum {
    BSP_SW_LEVEL_HIZ = 0,  /**< 引脚为高（外部上拉，开关断开）*/
    BSP_SW_LEVEL_LOW = 1,  /**< 引脚为低（开关接 GND）        */
} BspSwLevel_t;

/* ── API ──────────────────────────────────────────────────────────── */

/**
 * @brief 初始化开关引脚（无需配置上拉，外部硬件已提供）
 * @note  当前仅作为占位和文档说明，pin_mux.c 已完成 GPIO 输入配置
 */
void BSP_Switch_Init(void);

/**
 * @brief 读取 SWA 当前原始电平（无消抖）
 */
BspSwLevel_t BSP_Switch_ReadSWA(void);

/**
 * @brief 读取 SWB 当前原始电平（无消抖）
 */
BspSwLevel_t BSP_Switch_ReadSWB(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SWITCH_H */
