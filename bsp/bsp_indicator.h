/**
 * @file    bsp_indicator.h
 * @brief   RX_OUT / TX_OUT 状态指示输出 BSP 接口
 * @layer   BSP（板级支持层）
 *
 * 硬件引脚（来自 pin_mux.c）：
 *   RX_OUT → PTB_0（GPIOB pin0，GPIO 输出）
 *   TX_OUT → PTB_1（GPIOB pin1，GPIO 输出）
 *
 * 位置状态编码表（电机到达目标位置后输出）：
 * ┌──────────────┬──────────┬──────────┐
 * │  到达位置    │  TX_OUT  │  RX_OUT  │
 * ├──────────────┼──────────┼──────────┤
 * │  2驱         │   低     │   高     │
 * │  4驱         │   高     │   低     │
 * │  4驱锁死     │   高     │   高     │
 * └──────────────┴──────────┴──────────┘
 *
 * 运动中 / 未到位 / 错误状态由上层决定输出（默认全低）。
 */

#ifndef BSP_INDICATOR_H
#define BSP_INDICATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "pins_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 引脚定义 ─────────────────────────────────────────────────────── */
#define BSP_IND_GPIO       GPIOB      /**< RX_OUT/TX_OUT 共用 GPIOB */
#define BSP_IND_RX_PIN     (0U)       /**< RX_OUT: PTB_0            */
#define BSP_IND_TX_PIN     (1U)       /**< TX_OUT: PTB_1            */

/* ── 位置指示状态枚举 ─────────────────────────────────────────────── */
typedef enum {
    BSP_IND_NONE     = 0,  /**< 无效/运动中：TX=低, RX=低 */
    BSP_IND_2WD      = 1,  /**< 2驱：      TX=低, RX=高   */
    BSP_IND_4WD      = 2,  /**< 4驱：      TX=高, RX=低   */
    BSP_IND_4WD_LOCK = 3,  /**< 4驱锁死：  TX=高, RX=高   */
} BspIndState_t;

/* ── API ──────────────────────────────────────────────────────────── */

/**
 * @brief 初始化指示输出（引脚方向由 pin_mux 已配置为输出，此处置初值）
 * @note  须在 Board_Init() 之后调用；初始输出全低（BSP_IND_NONE）
 */
void BSP_Indicator_Init(void);

/**
 * @brief 设置指示输出状态
 * @param st  BspIndState_t 位置状态
 *
 * 根据状态编码表同时更新 TX_OUT 和 RX_OUT 两个引脚电平。
 */
void BSP_Indicator_Set(BspIndState_t st);

/**
 * @brief 获取当前指示状态
 */
BspIndState_t BSP_Indicator_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_INDICATOR_H */
