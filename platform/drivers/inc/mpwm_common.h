/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file mpwm_common.h
 * @version 1.4.1
 *
 * @brief MPWM Common Driver — Shared types, state, and base-level API.
 *
 * This header defines the common data types shared by all MPWM operating
 * modes, together with the base driver APIs for initialization, clock
 * selection, counter control, channel configuration, software trigger,
 * buffered-load control, interrupt-flag handling, and counter readback.
 *
 * The APIs are organized into the following categories:
 *   - Initialization & De-initialization
 *   - Clock Configuration
 *   - Counter Control
 *   - Channel Configuration
 *   - Software Trigger
 *   - Load Control (LDOK)
 *   - Status & Interrupt Flags
 *   - Counter Read
 *
 * @note Enable the MPWM peripheral clock before calling functions in this
 *       module.
 */

#ifndef MPWM_COMMON_H
#define MPWM_COMMON_H

#include "status.h"
#include "device_registers.h"
#include "clock_manager.h"
#include "interrupt_manager.h"
#include "mpwm_hw_access.h"
#include <stddef.h>

/*!
 * @defgroup mpwm_common_driver MPWM Common Driver
 * @ingroup mpwm
 * @brief Shared types and base-level APIs for the MPWM peripheral.
 * @details Provides the common enumerations, state structures, and
 *          instance-based helper APIs used by all MPWM operating modes:
 *          PWM, input capture, output compare, pulse counter, and common
 *          timer.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief MPWM clock source selection.
 *
 * | Value            | Clock Source                          |
 * |------------------|--------------------------------------|
 * | MPWM_BUS_CLOCK   | System bus clock.                    |
 * | MPWM_FUNC_CLOCK  | Functional (IPC) clock, when supported. |
 */
typedef enum
{
    MPWM_BUS_CLOCK = 0x0U,   /*!< Bus clock */
    MPWM_FUNC_CLOCK = 0x1U,  /*!< Functional clock */
} mpwm_clk_src_t;

/*!
 * @brief MPWM hardware channel index.
 *
 * The number of valid channels depends on the device-specific
 * `MPWM_CH_COUNT` definition.
 */
typedef enum
{
    MPWM_CHANNEL_0 = 0x0U,  /*!< channel 0 */
    MPWM_CHANNEL_1 = 0x1U,  /*!< channel 1 */
    MPWM_CHANNEL_2 = 0x2U,  /*!< channel 2 */
    MPWM_CHANNEL_3 = 0x3U,  /*!< channel 3 */
    MPWM_CHANNEL_4 = 0x4U,  /*!< channel 4 */
    MPWM_CHANNEL_5 = 0x5U,  /*!< channel 5 */
    MPWM_CHANNEL_6 = 0x6U,  /*!< channel 6 */
    MPWM_CHANNEL_7 = 0x7U,  /*!< channel 7 */
    MPWM_CHANNEL_8 = 0x8U,  /*!< channel 8  */
    MPWM_CHANNEL_9 = 0x9U,  /*!< channel 9  */
    MPWM_CHANNEL_10 = 0xAU,  /*!< channel 10 */
    MPWM_CHANNEL_11 = 0xBU,  /*!< channel 11 */
    MPWM_CHANNEL_12 = 0xCU,  /*!< channel 12 */
    MPWM_CHANNEL_13 = 0xDU,  /*!< channel 13 */
    MPWM_CHANNEL_14 = 0xEU,  /*!< channel 14 */
    MPWM_CHANNEL_15 = 0xFU,  /*!< channel 15 */
} mpwm_ch_index_t;

/*!
 * @brief MPWM channel operating mode.
 *
 * | Value                    | Mode            | Description                         |
 * |--------------------------|-----------------|-------------------------------------|
 * | MPWM_COMMON_TIMER_MODE   | Common Timer    | Free-running timer channel.         |
 * | MPWM_PWM_MODE            | PWM             | Edge-aligned PWM output.            |
 * | MPWM_OUTPUT_COMPARE_MODE | Output Compare  | Output action on compare match.     |
 * | MPWM_INPUT_CAPTURE_MODE  | Input Capture   | Capture counter on input edge.      |
 * | MPWM_PULSE_COUNTER_MODE  | Pulse Counter   | Count external input pulses.        |
 * | MPWM_CENTER_PWM_MODE     | Center PWM      | Center-aligned PWM output.          |
 */
typedef enum
{
    MPWM_COMMON_TIMER_MODE,   /*!< Common timer mode */
    MPWM_PWM_MODE,            /*!< PWM mode */
    MPWM_OUTPUT_COMPARE_MODE, /*!< Output compare mode */
    MPWM_INPUT_CAPTURE_MODE,  /*!< Input capture mode */
    MPWM_PULSE_COUNTER_MODE,  /*!< Pulse counter mode */
    MPWM_CENTER_PWM_MODE,     /*!< Center PWM mode */
} mpwm_ch_mode_t;

/*!
 * @brief MPWM channel clock prescaler.
 *
 * The channel counter clock equals the selected MPWM source clock divided by
 * `1 << prescaler`.
 *
 * | Value               | Division Factor |
 * |---------------------|-----------------|
 * | MPWM_CLK_DIV_BY_1   | /1              |
 * | MPWM_CLK_DIV_BY_2   | /2              |
 * | MPWM_CLK_DIV_BY_4   | /4              |
 * | MPWM_CLK_DIV_BY_8   | /8              |
 * | MPWM_CLK_DIV_BY_16  | /16             |
 * | MPWM_CLK_DIV_BY_32  | /32             |
 * | MPWM_CLK_DIV_BY_64  | /64             |
 * | MPWM_CLK_DIV_BY_128 | /128            |
 */
typedef enum
{
    MPWM_CLK_DIV_BY_1 = 0x0U,  /*!< Divide by 1 */
    MPWM_CLK_DIV_BY_2 = 0x1U,  /*!< Divide by 2 */
    MPWM_CLK_DIV_BY_4 = 0x2U,  /*!< Divide by 4 */
    MPWM_CLK_DIV_BY_8 = 0x3U,  /*!< Divide by 8 */
    MPWM_CLK_DIV_BY_16 = 0x4U,  /*!< Divide by 16 */
    MPWM_CLK_DIV_BY_32 = 0x5U,  /*!< Divide by 32 */
    MPWM_CLK_DIV_BY_64 = 0x6U,  /*!< Divide by 64 */
    MPWM_CLK_DIV_BY_128 = 0x7U,  /*!< Divide by 128 */
} mpwm_ch_clk_prs_t;

/*!
 * @brief MPWM channel reload source.
 *
 * Selects when buffered PERIOD/CMP values are transferred into the active
 * channel registers.
 *
 * | Value                  | Reload Trigger                           |
 * |------------------------|------------------------------------------|
 * | MPWM_RELOAD_ON_PERIOD  | Reload on period event.                  |
 * | MPWM_RELOAD_ON_HW_TRIG | Reload on hardware trigger.              |
 * | MPWM_RELOAD_ON_SW_TRIG | Reload on software trigger.              |
 * | MPWM_RELOAD_DISABLE    | Disable buffering; update takes effect immediately. |
 */
typedef enum
{
    MPWM_RELOAD_ON_PERIOD,  /*!< Reload when count is equal to period */
    MPWM_RELOAD_ON_HW_TRIG,  /*!< Reload when hardware trigger arrives */
    MPWM_RELOAD_ON_SW_TRIG,  /*!< Reload when software trigger arrives */
    MPWM_RELOAD_DISABLE,  /*!< Reload mechanism disabled, it activates immediately */
} mpwm_ch_reload_src_t;

/*!
 * @brief MPWM channel counting mode.
 *
 * | Value                | Behavior                                       |
 * |----------------------|------------------------------------------------|
 * | MPWM_CONTINUOUS_MODE | Counter wraps and continues after overflow.    |
 * | MPWM_ONESHOT_MODE    | Counter stops after the first overflow event.  |
 */
typedef enum
{
    MPWM_CONTINUOUS_MODE,  /*!< counter continuous mode */
    MPWM_ONESHOT_MODE,  /*!< Counter one shot mode */
} mpwm_ch_count_mode_t;

/*!
 * @brief MPWM channel output initial level.
 *
 * This value is driven before the counter starts and after mode teardown.
 *
 * | Value              | Output Level |
 * |--------------------|--------------|
 * | MPWM_OUTPUT_INIT_0 | Low          |
 * | MPWM_OUTPUT_INIT_1 | High         |
 */
typedef enum
{
    MPWM_OUTPUT_INIT_0,  /*!< Output initial value is 0 */
    MPWM_OUTPUT_INIT_1,  /*!< Output initial value is 1 */
} mpwm_ch_output_init_t;

/*!
 * @brief MPWM output polarity selection.
 *
 * | Value           | Effect                        |
 * |-----------------|-------------------------------|
 * | MPWM_NOT_INVERT | Normal output polarity.       |
 * | MPWM_INVERT     | Inverted output polarity.     |
 */
typedef enum
{
    MPWM_NOT_INVERT,  /*!< Not invert output */
    MPWM_INVERT,  /*!< Invert output */
} mpwm_ch_polarity_t;

/*!
 * @brief PWM waveform alignment mode.
 *
 * | Value                   | Alignment Meaning                 |
 * |-------------------------|-----------------------------------|
 * | MPWM_PWM_RIGHT_ALIGN    | Edge-aligned, active edge at end. |
 * | MPWM_PWM_LEFT_ALIGN     | Edge-aligned, active edge at start. |
 * | MPWM_PWM_CENTER_H_ALIGN | Center-aligned, high-true.        |
 * | MPWM_PWM_CENTER_L_ALIGN | Center-aligned, low-true.         |
 */
typedef enum
{
    MPWM_PWM_RIGHT_ALIGN,          /*!< PWM Right alignment */
    MPWM_PWM_LEFT_ALIGN,           /*!< PWM Left alignment */
    MPWM_PWM_CENTER_H_ALIGN,       /*!< PWM Center High alignment */
    MPWM_PWM_CENTER_L_ALIGN,       /*!< PWM Center Low alignment */
} mpwm_ch_pwm_align_mode_t;

/*!
 * @brief MPWM trigger-output enable configuration.
 *
 * | Field           | Description                                      |
 * |-----------------|--------------------------------------------------|
 * | cmpTrigOutputEn | Generate a trigger pulse on compare match.       |
 * | perTrigOutputEn | Generate a trigger pulse on period/overflow.     |
 */
typedef struct
{
    bool cmpTrigOutputEn;  /*!< Trigger output enable when matching compare */
    bool perTrigOutputEn;  /*!< Trigger output enable when matching period */
} mpwm_ch_output_trig_config_t;

/*!
 * @brief PWM duty-cycle update unit.
 *
 * | Value                       | Interpretation                    |
 * |-----------------------------|-----------------------------------|
 * | MPWM_UPDATE_IN_DUTY_CYCLE   | Duty cycle is fixed-point 0..0x8000. |
 * | MPWM_UPDATE_IN_TICKS        | Duty cycle is a raw tick count.   |
 */
typedef enum
{
    MPWM_UPDATE_IN_DUTY_CYCLE,  /*!< Update in duty cycle, like 0x1000, 0x4000 and so on */
    MPWM_UPDATE_IN_TICKS,  /*!< Update in ticks */
} mpwm_pwm_update_unit_t;

/*!
 * @brief Period argument unit used by PWM update APIs.
 *
 * | Value                | Unit                     |
 * |----------------------|--------------------------|
 * | MPWM_PERIOD_IN_HZ    | Frequency in Hz.         |
 * | MPWM_PERIOD_IN_TICKS | Raw period tick count.   |
 */
typedef enum
{
    MPWM_PERIOD_IN_HZ = 0U, /*!< PWM period in Hz */
    MPWM_PERIOD_IN_TICKS = 1U, /*!< PWM period in ticks */
} mpwm_period_unit_t;

/*!
 * @brief Runtime state of an MPWM instance.
 *
 * The application allocates this structure and passes it to
 * `MPWM_DRV_Init()`. The driver updates it as channels are configured and
 * modified at runtime.
 *
 * | Field            | Type                       | Description                          |
 * |------------------|----------------------------|--------------------------------------|
 * | mpwmClkSrc       | mpwm_clk_src_t             | Active source clock.                 |
 * | mpwmClkSrcFreq   | uint32_t                   | Source clock frequency in Hz.        |
 * | mpwmChMode       | mpwm_ch_mode_t[]           | Operating mode for each channel.     |
 * | mpwmChAlignMode  | mpwm_ch_pwm_align_mode_t[] | PWM alignment of each channel.       |
 * | mpwmChFreq       | uint32_t[]                 | Channel counter clock in Hz.         |
 * | mpwmChPeriodTicks| uint16_t[]                 | Channel period value in ticks.       |
 * | mpwmChDutyCycle  | uint16_t[]                 | Last programmed duty-cycle value.    |
 * | mpwmChUpdateUnit | mpwm_pwm_update_unit_t[]   | Duty-cycle unit for each channel.    |
 */
typedef struct
{
    mpwm_clk_src_t mpwmClkSrc;                                  /*!< The clock source */
    uint32_t mpwmClkSrcFreq;                                    /*!< The clock source frequency in Hz */
    mpwm_ch_mode_t mpwmChMode[MPWM_CH_COUNT];                   /*!< The channel mode */
    mpwm_ch_pwm_align_mode_t mpwmChAlignMode[MPWM_CH_COUNT];    /*!< The channel align mode */
    uint32_t mpwmChFreq[MPWM_CH_COUNT];                         /*!< The channel counter frequency in Hz */
    uint16_t mpwmChPeriodTicks[MPWM_CH_COUNT];                  /*!< The channel frequency ticks */
    uint16_t mpwmChDutyCycle[MPWM_CH_COUNT];                    /*!< The channel duty cycle (0 ~ 0x8000) */
    mpwm_pwm_update_unit_t mpwmChUpdateUnit[MPWM_CH_COUNT];     /*!< The channel update unit */
} mpwm_state_t;

/*!
 * @brief MPWM instance configuration structure.
 *
 * | Field  | Type           | Description                  |
 * |--------|----------------|------------------------------|
 * | clkSrc | mpwm_clk_src_t | MPWM source-clock selection. |
 */
typedef struct
{
    mpwm_clk_src_t clkSrc;     /*!< MPWM peripheral clock source */
} mpwm_user_config_t;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*! @brief Table of base addresses for MPWM peripheral instances. */
extern MPWM_Type *g_mpwmBase[MPWM_INSTANCE_COUNT];

/*! @brief Table of overflow IRQ numbers for each MPWM channel. */
extern IRQn_Type g_mpwmOvfIrqId[MPWM_INSTANCE_COUNT][MPWM_CH_COUNT];

/*! @brief Table of channel event IRQ numbers for each MPWM channel. */
extern IRQn_Type g_mpwmChnIrqId[MPWM_INSTANCE_COUNT][MPWM_CH_COUNT];

/*! @brief Driver state pointers for each MPWM instance. */
extern mpwm_state_t *g_mpwmState[MPWM_INSTANCE_COUNT];

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization & De-initialization
 * @brief Functions for setting up and tearing down an MPWM instance.
 * @{
 */

/*!
 * @brief Initialize the MPWM peripheral and driver state.
 *
 * Resets the peripheral, initializes the state structure, and selects the
 * default bus clock.
 *
 * @param[in]  instance  MPWM instance index (0-based).
 * @param[out] state     Pointer to the driver state structure.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Initialization completed successfully.
 * @retval STATUS_ERROR    The instance is already initialized.
 *
 * @pre Enable the MPWM peripheral clock before calling this function.
 */
status_t MPWM_DRV_Init(uint32_t instance, mpwm_state_t *state);

/*!
 * @brief De-initialize the MPWM peripheral.
 *
 * Resets MPWM registers and releases the stored driver state pointer.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @return Execution status.
 * @retval STATUS_SUCCESS  De-initialization completed successfully.
 */
status_t MPWM_DRV_Deinit(uint32_t instance);

/*! @} */

/*!
 * @name Clock Configuration
 * @brief Functions for selecting and querying the MPWM source clock.
 * @{
 */

/*!
 * @brief Select the MPWM peripheral clock source.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] clkSrc    Clock source selection.
 * @return Execution status.
 * @retval STATUS_SUCCESS                  Clock source updated successfully.
 * @retval STATUS_MPWM_ERROR_FUNC_CLK_FREQ Functional clock exceeds the
 *                                         allowed device limit.
 *
 * @warning When `MPWM_FUNC_CLOCK` is selected, the functional clock must be
 *          less than or equal to `FAST_BUS_CLK / 2`.
 */
status_t MPWM_DRV_SetClkSrc(uint32_t instance, mpwm_clk_src_t clkSrc);

/*!
 * @brief Get the active MPWM clock source.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @return Active clock source.
 */
mpwm_clk_src_t MPWM_DRV_GetClkSrc(uint32_t instance);

/*!
 * @brief Get the active MPWM source-clock frequency.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @return Clock frequency in Hz.
 */
uint32_t MPWM_DRV_GetFrequency(uint32_t instance);

/*! @} */

/*!
 * @name Counter Control
 * @brief Functions for enabling and disabling channel counters.
 * @{
 */

/*!
 * @brief Enable counters for multiple channels.
 *
 * @param[in] instance    MPWM instance index (0-based).
 * @param[in] channelSet  Channel-enable bitmask.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Counters enabled successfully.
 */
status_t MPWM_DRV_EnableCounter(uint32_t instance, uint16_t channelSet);

/*!
 * @brief Disable counters for multiple channels.
 *
 * @param[in] instance    MPWM instance index (0-based).
 * @param[in] channelSet  Channel-disable bitmask.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Counters disabled successfully.
 */
status_t MPWM_DRV_DisableCounter(uint32_t instance, uint16_t channelSet);

/*! @} */

/*!
 * @name Channel Configuration
 * @brief Functions for per-channel reload, output, DMA, period, and compare
 *        configuration.
 * @{
 */

/*!
 * @brief Set the reload source for a channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @param[in] src       Reload source selection.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Reload source updated successfully.
 */
status_t MPWM_DRV_SetReloadSrc(uint32_t instance,
                               uint8_t channel,
                               mpwm_ch_reload_src_t src);

/*!
 * @brief Get the reload source of a channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @return Current reload source.
 */
mpwm_ch_reload_src_t MPWM_DRV_GetReloadSrc(uint32_t instance, uint8_t channel);

/*!
 * @brief Set the output initial value for a single channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @param[in] initVal   Output initial value.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Output initial value updated successfully.
 */
status_t MPWM_DRV_SetChannelOutputInit(uint32_t instance,
                                       uint8_t channel,
                                       mpwm_ch_output_init_t initVal);

/*!
 * @brief Set output initial values for all channels in one write.
 *
 * @param[in] instance    MPWM instance index (0-based).
 * @param[in] initValSet  Bitmask of initial values.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Output initial values updated successfully.
 */
status_t MPWM_DRV_SetOutputInit(uint32_t instance, uint16_t initValSet);

/*!
 * @brief Get the output-initial-value bitmask of all channels.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @return Output initial value bitmask.
 */
uint16_t MPWM_DRV_GetOutputInit(uint32_t instance);

/*!
 * @brief Set output polarity for all channels in one write.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] polSet    Output polarity bitmask.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Output polarity updated successfully.
 */
status_t MPWM_DRV_SetPolarity(uint32_t instance, uint16_t polSet);

/*!
 * @brief Get the output-polarity bitmask of all channels.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @return Output polarity bitmask.
 */
uint16_t MPWM_DRV_GetPolarity(uint32_t instance);

/*!
 * @brief Set the output polarity of a single channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @param[in] pol       Output polarity.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Channel polarity updated successfully.
 */
status_t MPWM_DRV_SetChannelPolarity(uint32_t instance,
                                     uint8_t channel,
                                     mpwm_ch_polarity_t pol);

/*!
 * @brief Enable DMA requests for a channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @return Execution status.
 * @retval STATUS_SUCCESS  DMA request generation enabled.
 */
status_t MPWM_DRV_EnableDma(uint32_t instance, uint8_t channel);

/*!
 * @brief Set the channel period value in ticks.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @param[in] value     Period value in ticks.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Period value updated successfully.
 */
status_t MPWM_DRV_SetChannelPeriod(uint32_t instance, uint8_t channel, uint16_t value);

/*!
 * @brief Set the channel compare value in ticks.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @param[in] value     Compare value in ticks.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Compare value updated successfully.
 */
status_t MPWM_DRV_SetChannelCompare(uint32_t instance, uint8_t channel, uint16_t value);

/*!
 * @brief Convert a channel frequency target into period ticks.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @param[in] freq      Target output frequency in Hz.
 * @return Equivalent period value in ticks.
 */
uint32_t MPWM_DRV_ConvertFreqToPeriodTicks(uint32_t instance,
                                           uint8_t channel,
                                           uint32_t freq);

/*! @} */

/*!
 * @name Software Trigger
 * @brief Functions for issuing software reload triggers.
 * @{
 */

/*!
 * @brief Issue a software trigger for a single channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Software trigger issued successfully.
 */
status_t MPWM_DRV_SetChannelSoftwareTrigger(uint32_t instance, uint8_t channel);

/*!
 * @brief Issue software triggers for multiple channels.
 *
 * @param[in] instance    MPWM instance index (0-based).
 * @param[in] channelSet  Channel bitmask to trigger.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Software triggers issued successfully.
 */
status_t MPWM_DRV_SetSoftwareTrigger(uint32_t instance, uint16_t channelSet);

/*! @} */

/*!
 * @name Status & Interrupt Flags
 * @brief Functions for reading and clearing MPWM status flags.
 * @{
 */

/*!
 * @brief Get the timer-overflow interrupt flag of a channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @return `true` if the overflow flag is asserted; otherwise `false`.
 */
bool MPWM_DRV_GetChannelTofIntFlag(uint32_t instance, uint8_t channel);

/*!
 * @brief Clear the timer-overflow interrupt flag of a channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 */
void MPWM_DRV_ClearChannelTofIntFlag(uint32_t instance, uint8_t channel);

/*!
 * @brief Get the channel-event interrupt flag.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @return `true` if the event flag is asserted; otherwise `false`.
 */
bool MPWM_DRV_GetChannelIntFlag(uint32_t instance, uint8_t channel);

/*!
 * @brief Clear the channel-event interrupt flag.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 */
void MPWM_DRV_ClearChannelIntFlag(uint32_t instance, uint8_t channel);

/*! @} */

/*!
 * @name Load Control (LDOK)
 * @brief Functions for buffered register update control.
 * @{
 */

/*!
 * @brief Clear the LDOK flag for a single channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 */
void MPWM_DRV_ClearChannelLdok(uint32_t instance, uint8_t channel);

/*!
 * @brief Clear the LDOK flag for multiple channels.
 *
 * @param[in] instance    MPWM instance index (0-based).
 * @param[in] channelSet  Channel bitmask.
 */
void MPWM_DRV_ClearLdok(uint32_t instance, uint16_t channelSet);

/*!
 * @brief Assert the LDOK flag for a single channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 */
void MPWM_DRV_SetChannelLdok(uint32_t instance, uint8_t channel);

/*!
 * @brief Assert the LDOK flag for multiple channels.
 *
 * @param[in] instance    MPWM instance index (0-based).
 * @param[in] channelSet  Channel bitmask.
 */
void MPWM_DRV_SetLdok(uint32_t instance, uint16_t channelSet);

/*! @} */

/*!
 * @name Counter Read
 * @brief Functions for reading captured and current counter values.
 * @{
 */

/*!
 * @brief Read the captured count value of a channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @return Captured counter value.
 */
uint32_t MPWM_DRV_ReadCaptureCount(uint32_t instance, uint8_t channel);

/*!
 * @brief Read the current counter value of a channel.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] channel   Channel index.
 * @return Current counter value.
 */
uint16_t MPWM_DRV_ReadCurrentCount(uint32_t instance, uint8_t channel);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* MPWM_COMMON_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
