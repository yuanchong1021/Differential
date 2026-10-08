/**
 * @file    bsp_indicator.c
 * @brief   RX_OUT / TX_OUT 状态指示输出 BSP 实现
 *
 * 位置状态编码：
 *   2驱      → TX_OUT=0, RX_OUT=1
 *   4驱      → TX_OUT=1, RX_OUT=0
 *   4驱锁死  → TX_OUT=1, RX_OUT=1
 *   无效     → TX_OUT=0, RX_OUT=0
 */

#include "bsp_indicator.h"

static BspIndState_t s_curState = BSP_IND_NONE;

/* ── 内部：同时写 TX_OUT 和 RX_OUT 两个引脚 ─────────────────────── */
/**
 * @brief 同步写入两路位置编码输出。
 *
 * 两个 GPIO 是对外位置状态编码，不参与电机控制。状态机只有在确认到位时
 * 才输出有效编码，运动中、未知位置和错误状态均输出 00。
 */
static void WriteOutputs(uint8_t txLevel, uint8_t rxLevel)
{
    PINS_DRV_WritePin(BSP_IND_GPIO, BSP_IND_TX_PIN, txLevel);
    PINS_DRV_WritePin(BSP_IND_GPIO, BSP_IND_RX_PIN, rxLevel);
}

/* ── 公共 API ─────────────────────────────────────────────────────── */

void BSP_Indicator_Init(void)
{
    /* pin_mux.c 已将 PTB_0 / PTB_1 配为 GPIO 输出，初值 0。此处显式置低。*/
    WriteOutputs(0U, 0U);
    s_curState = BSP_IND_NONE;
}

void BSP_Indicator_Set(BspIndState_t st)
{
    if (st == s_curState) { return; }  /* 状态未变，不重复写 */

    switch (st)
    {
        case BSP_IND_2WD:
            /* 2驱：TX=0, RX=1 */
            WriteOutputs(0U, 1U);
            break;

        case BSP_IND_4WD:
            /* 4驱：TX=1, RX=0 */
            WriteOutputs(1U, 0U);
            break;

        case BSP_IND_4WD_LOCK:
            /* 4驱锁死：TX=1, RX=1 */
            WriteOutputs(1U, 1U);
            break;

        case BSP_IND_NONE:
        default:
            /* 无效/运动中：TX=0, RX=0 */
            WriteOutputs(0U, 0U);
            break;
    }

    s_curState = st;
}

BspIndState_t BSP_Indicator_Get(void)
{
    return s_curState;
}
