/*
 * Copyright 2020-2023 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file mpwm_hw_access.h
 * @version 1.4.1
 *
 * @brief MPWM HW Access Layer — inline register-level helper functions.
 *
 * This internal header provides static-inline helpers for direct access to the
 * MPWM peripheral registers. These helpers are used by the MPWM driver layer
 * to configure interrupts, output state, reload behavior, trigger sources,
 * channel operating modes, clock prescalers, and timing registers.
 */

#ifndef MPWM_HW_ACCESS_H
#define MPWM_HW_ACCESS_H

#include "status.h"
#include "device_registers.h"

/*!
 * @addtogroup mpwm_driver
 * @brief MPWM hardware access helper layer.
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * Code definition
*******************************************************************************/

/*!
 * @brief Set the timer-overflow interrupt enable state of one MPWM channel.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] ch    The MPWM channel index.
 * @param[in] en    The timer-overflow interrupt enable state.
 */
static inline void MPWM_SetChannelTofIntEnState(MPWM_Type *base, uint8_t ch, bool en)
{
    base->INTE0 &= ~((uint32_t)MPWM_INTE0_CH0TOIE_MASK << ch);
    base->INTE0 |= ((uint32_t)(en ? 1U : 0U) << ch);
}

/*!
 * @brief Set the channel interrupt enable state of one MPWM channel.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] ch    The MPWM channel index.
 * @param[in] en    The channel interrupt enable state.
 */
static inline void MPWM_SetChannelIntEnState(MPWM_Type *base, uint8_t ch, bool en)
{
    base->INTE1 &= ~((uint32_t)MPWM_INTE1_CH0IE_MASK << ch);
    base->INTE1 |= ((uint32_t)(en ? 1U : 0U) << ch);
}

/*!
 * @brief Write the interrupt-enable bitmap for all MPWM channels.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] val   The interrupt-enable bitmap.
 */
static inline void MPWM_SetIntEn(MPWM_Type *base, uint32_t val)
{
    base->INTE1 = val;
}

/*!
 * @brief Enable the counters of a set of MPWM channels.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] val   The bitmask of enabled channel(s).
 */
static inline void MPWM_EnableCounter(MPWM_Type *base, uint32_t val)
{
    base->CTRL |= val;
}

/*!
 * @brief Disable the counters of a set of MPWM channels.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] val   The bitmask of disabled channel(s).
 */
static inline void MPWM_DisableCounter(MPWM_Type *base, uint32_t val)
{
    base->CTRL &= ~val;
}

/*!
 * @brief Trigger a software update for multiple MPWM channels.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] val   The bitmask of channels to trigger.
 */
static inline void MPWM_SetSwTrig(MPWM_Type *base, uint32_t val)
{
    base->SWT = val;
}

/*!
 * @brief Trigger a software update for one MPWM channel.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] ch    The MPWM channel index.
 */
static inline void MPWM_SetChannelSwTrig(MPWM_Type *base, uint8_t ch)
{
    base->SWT = ((uint32_t)0x1U << ch);
}

/*!
 * @brief Set the output initial value of one MPWM channel.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] ch    The MPWM channel index.
 * @param[in] val   The output initial value.
 */
static inline void MPWM_SetChannelOutputInit(MPWM_Type *base, uint8_t ch, uint8_t val)
{
    if (val == 0U)
    {
        base->INIT &= ~((uint32_t)0x1U << ch);
    }
    else
    {
        base->INIT |= ((uint32_t)0x1U << ch);
    }
}

/*!
 * @brief Set the output initial values of all MPWM channels.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] val   The bitmask of output initial values.
 */
static inline void MPWM_SetOutputInit(MPWM_Type *base, uint32_t val)
{
    base->INIT = val;
}

/*!
 * @brief Get the output initial-value bitmask of all MPWM channels.
 *
 * @param[in] base  The MPWM base address pointer.
 * @return Output initial-value bitmask.
 */
static inline uint32_t MPWM_GetOutputInit(MPWM_Type *base)
{
    return base->INIT;
}

/*!
 * @brief Set the output polarity of one MPWM channel.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] ch    The MPWM channel index.
 * @param[in] val   The polarity value.
 */
static inline void MPWM_SetChannelPolarity(MPWM_Type *base, uint8_t ch, uint8_t val)
{
    if (val == 0U)
    {
        base->POL &= ~((uint32_t)0x1U << ch);
    }
    else
    {
        base->POL |= ((uint32_t)0x1U << ch);
    }
}

/*!
 * @brief Set the output polarity bitmask of all MPWM channels.
 *
 * @param[in] base  The MPWM base address pointer.
 * @param[in] val   The polarity bitmask.
 */
static inline void MPWM_SetPolarity(MPWM_Type *base, uint32_t val)
{
    base->POL = val;
}

/*!
 * @brief Get the polarity bitmask of all MPWM channels.
 *
 * @param[in] base  The MPWM base address pointer.
 * @return Polarity bitmask.
 */
static inline uint32_t MPWM_GetPolarity(MPWM_Type *base)
{
    return base->POL;
}

#if FEATURE_MPWM_HAS_COMBINATION_CAPTURE
/*!
 * @brief Set MPWM channel pair combination source
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] val  The set of all pairs' combination source
 */
static inline void MPWM_SetChPairCombSrc(MPWM_Type *base, uint32_t val)
{
    base->COMBCMP |= val << 16;
}

/*!
 * @brief Set MPWM channel pair combination enable state
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] val  The set of all pairs' combination enable state
 */
static inline void MPWM_SetChPairCombEnState(MPWM_Type *base, uint32_t val)
{
    base->COMBCMP |= val;
}
#endif

/*!
 * @brief Set MPWM channel counter reset to 0 automatically when new
 *        capture event occurs.
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] en   Auto clear is enabled or not
 */
static inline void MPWM_SetAutoClearEnState(MPWM_Type *base,
                                            uint8_t ch,
                                            bool en)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_ACLR_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_ACLR(en ? 1U : 0U);
}

/*!
 * @brief Set MPWM channel overwrite CH_CAP register when new capture
 *        event occurs.
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] en   Overwrite is enabled or not
 */
static inline void MPWM_SetOverWriteEnState(MPWM_Type *base,
                                            uint8_t ch,
                                            bool en)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_OVWR_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_OVWR(en ? 1U : 0U);
}

/*!
 * @brief Set MPWM channel capture edge
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] edge The capture edge
 */
static inline void MPWM_SetCapEdge(MPWM_Type *base, uint8_t ch, uint8_t edge)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_CAPEDGE_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_CAPEDGE(edge);
}

/*!
 * @brief Set MPWM channel output compare mode
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] mode The output compare mode
 */
static inline void MPWM_SetCmpMode(MPWM_Type *base, uint8_t ch, uint8_t mode)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_CMPOUT_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_CMPOUT(mode);
}

/*!
 * @brief Set MPWM channel reload source
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] src  The reload source
 */
static inline void MPWM_SetReloadSrc(MPWM_Type *base, uint8_t ch, uint8_t src)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_RLDSRC_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_RLDSRC(src);
}

/*!
 * @brief Get MPWM channel reload source
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @return source
 */
static inline uint8_t MPWM_GetReloadSrc(MPWM_Type *base, uint8_t ch)
{
    return (uint8_t)((base->CH[ch].CFG & MPWM_CH_CFG_RLDSRC_MASK) >> MPWM_CH_CFG_RLDSRC_SHIFT);
}

/*!
 * @brief Set MPWM channel dma enable or not
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] en   The dma enable or not
 */
static inline void MPWM_SetDmaEnState(MPWM_Type *base, uint8_t ch, bool en)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_DMAEN_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_DMAEN(en ? 1U : 0U);
}

/*!
 * @brief Set MPWM channel debug mode disable or not
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] en   The debug mode disable or not
 */
static inline void MPWM_SetDebugMode(MPWM_Type *base, uint8_t ch, bool en)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_DBGDIS_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_DBGDIS(en ? 0U : 1U);
}

#if FEATURE_MPWM_HAS_SIMULTANEOUS_UPDATE
/*!
 * @brief Clear MPWM channel LDOK
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 */
static inline void MPWM_ClearLdok(MPWM_Type *base, uint8_t ch)
{
    base->CLDOK = (uint32_t)0x1U << ch;
}

/*!
 * @brief Set MPWM channel LDOK
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 */
static inline void MPWM_SetLdok(MPWM_Type *base, uint8_t ch)
{
    base->LDOK = (uint32_t)0x1U << ch;
}
#else
/*!
 * @brief Clear MPWM channel LDOK
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 */
static inline void MPWM_ClearLdok(MPWM_Type *base, uint8_t ch)
{
    base->CH[ch].CFG |= MPWM_CH_CFG_CLDOK(1);
}

/*!
 * @brief Set MPWM channel LDOK
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 */
static inline void MPWM_SetLdok(MPWM_Type *base, uint8_t ch)
{
    base->CH[ch].CFG |= MPWM_CH_CFG_LDOK(1);
}
#endif

/*!
 * @brief Set MPWM channel hardware trigger enable or not
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] en   Hardware trigger is enabled or not
 */
static inline void MPWM_SetHwTrigEnState(MPWM_Type *base, uint8_t ch, bool en)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_HWTRIG_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_HWTRIG(en ? 1U : 0U);
}

/*!
 * @brief Set MPWM channel output trigger on compare or not
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] en   enabled or disabled
 */
static inline void MPWM_SetCmpOutputTrigEnState(MPWM_Type *base, uint8_t ch, bool en)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_CMPTOEN_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_CMPTOEN(en ? 1U : 0U);
}

/*!
 * @brief Set MPWM channel output trigger on period or not
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] en   enabled or disabled
 */
static inline void MPWM_SetPerOutputTrigEnState(MPWM_Type *base, uint8_t ch, bool en)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_PERTOEN_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_PERTOEN(en ? 1U : 0U);
}

/*!
 * @brief Set MPWM channel clock prescaler
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] prs  The clock prescaler
 */
static inline void MPWM_SetClkPrs(MPWM_Type *base, uint8_t ch, uint8_t prs)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_CLKPRS_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_CLKPRS(prs);
}

/*!
 * @brief Set MPWM channel counter count mode
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] mode The counter count mode
 */
static inline void MPWM_SetCountMode(MPWM_Type *base, uint8_t ch, uint8_t mode)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_ONESHOT_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_ONESHOT(mode);
}

/*!
 * @brief Set MPWM channel mode
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] mode The channel mode
 */
static inline void MPWM_SetChannelMode(MPWM_Type *base, uint8_t ch, uint8_t mode)
{
    base->CH[ch].CFG &= ~MPWM_CH_CFG_CHMODE_MASK;
    base->CH[ch].CFG |= MPWM_CH_CFG_CHMODE(mode);
}

/*!
 * @brief Get MPWM channel counter current count
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @return count
 */
static inline uint32_t MPWM_GetCurrentCount(MPWM_Type *base, uint8_t ch)
{
    return base->CH[ch].CNT;
}

/*!
 * @brief Set MPWM channel period(counter max value)
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] val  The channel period(counter max value)
 */
static inline void MPWM_SetChannelPeriod(MPWM_Type *base, uint8_t ch, uint32_t val)
{
    base->CH[ch].PERIOD = val;
}

/*!
 * @brief Get MPWM channel period ticks
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @return period
 */
static inline uint32_t MPWM_GetChannelPeriod(MPWM_Type *base, uint8_t ch)
{
    return base->CH[ch].PERIOD;
}

/*!
 * @brief Set MPWM channel compare
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @param[in] val  The channel compare
 */
static inline void MPWM_SetChannelCompare(MPWM_Type *base, uint8_t ch, uint32_t val)
{
    base->CH[ch].CMP = val;
}

/*!
 * @brief Get MPWM channel compare ticks
 *
 * @param[in] base The MPWM base address pointer
 * @param[in] ch   The MPWM channel
 * @return compare
 */
static inline uint32_t MPWM_GetChannelCompare(MPWM_Type *base, uint8_t ch)
{
    return base->CH[ch].CMP;
}




#if defined(__cplusplus)
}
#endif


/*! @}*/


#endif /* MPWM_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
