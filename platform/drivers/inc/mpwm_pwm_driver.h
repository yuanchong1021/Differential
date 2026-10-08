/*
 * Copyright 2020-2023 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file mpwm_pwm_driver.h
 * @version 1.4.1
 *
 * @brief MPWM PWM Driver — Public API for PWM output generation.
 *
 * This header defines the configuration structures and driver APIs for using
 * the MPWM peripheral in PWM output mode. It supports edge-aligned and
 * center-aligned PWM waveforms with configurable frequency, duty cycle,
 * polarity, reload behavior, and trigger output.
 *
 * The APIs are organized into two categories:
 *   - Initialization & De-initialization
 *   - Runtime Update
 *
 * @note Call `MPWM_DRV_Init()` before using any function in this module.
 */

#ifndef MPWM_PWM_DRIVER_H
#define MPWM_PWM_DRIVER_H

#include "mpwm_common.h"

/*!
 * @defgroup mpwm_pwm_driver MPWM PWM Driver
 * @ingroup mpwm
 * @brief PWM output mode driver for the MPWM peripheral.
 * @details Provides APIs for configuring channels in PWM mode and for
 *          updating period and duty-cycle parameters at runtime.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Maximum duty-cycle value when `MPWM_UPDATE_IN_DUTY_CYCLE` is used. */
#define MPWM_MAX_DUTY_CYCLE            (0x8000U)

/*! @brief Fixed-point shift used when converting duty-cycle values to ticks. */
#define MPWM_DUTY_TO_TICKS_SHIFT       (15)

/*******************************************************************************
 * Data Types
 ******************************************************************************/

/*!
 * @brief Per-channel configuration for PWM mode.
 *
 * | Field              | Type                          | Description                               |
 * |--------------------|-------------------------------|-------------------------------------------|
 * | hwChannelId        | uint8_t                       | Hardware channel index.                   |
 * | clkPrs             | mpwm_ch_clk_prs_t             | Channel counter prescaler.                |
 * | frequencyHz        | uint32_t                      | Target PWM frequency in hertz.            |
 * | unit               | mpwm_pwm_update_unit_t        | Duty-cycle input unit.                    |
 * | dutyCyclePercent   | uint16_t                      | Duty cycle in fixed-point or ticks.       |
 * | alignMode          | mpwm_ch_pwm_align_mode_t      | PWM alignment mode.                       |
 * | outputInit         | mpwm_ch_output_init_t         | Initial output level before start.        |
 * | debugMode          | bool                          | Keep running while the core is halted.    |
 * | reloadSrc          | mpwm_ch_reload_src_t          | Buffered-register reload source.          |
 * | outputTrigConfig   | mpwm_ch_output_trig_config_t  | Compare/period trigger-output enables.    |
 * | intEn              | bool                          | Enable channel-event interrupt.           |
 * | timerOverflowIntEn | bool                          | Enable timer-overflow interrupt.          |
 */
typedef struct
{
    uint8_t hwChannelId;
    mpwm_ch_clk_prs_t clkPrs;
    uint32_t frequencyHz;
    mpwm_pwm_update_unit_t unit;
    uint16_t dutyCyclePercent;
    mpwm_ch_pwm_align_mode_t alignMode;
    mpwm_ch_output_init_t outputInit;
    bool debugMode;
    mpwm_ch_reload_src_t reloadSrc;
    mpwm_ch_output_trig_config_t outputTrigConfig;
    bool intEn;
    bool timerOverflowIntEn;
} mpwm_pwm_ch_config_t;

/*!
 * @brief PWM-mode configuration for one MPWM instance.
 *
 * | Field            | Type                    | Description                             |
 * |------------------|-------------------------|-----------------------------------------|
 * | channelNum       | uint8_t                 | Number of configured channels.          |
 * | pwmChannelConfig | mpwm_pwm_ch_config_t *  | Pointer to the channel-configuration array. |
 */
typedef struct
{
    uint8_t channelNum;
    mpwm_pwm_ch_config_t *pwmChannelConfig;
} mpwm_pwm_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name Initialization & De-initialization
 * @brief Functions for setting up and tearing down PWM mode.
 * @{
 */

/*!
 * @brief Initialize MPWM channels in PWM output mode.
 *
 * Configures one or more channels for PWM output. After initialization,
 * call `MPWM_DRV_EnableCounter()` to start waveform generation.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] config    Pointer to the PWM configuration structure.
 * @return Execution status.
 * @retval STATUS_SUCCESS  PWM mode initialized successfully.
 * @retval STATUS_ERROR    A duty-cycle value is outside the accepted range.
 */
status_t MPWM_DRV_InitPwm(uint32_t instance, const mpwm_pwm_config_t *config);

/*!
 * @brief De-initialize PWM mode for the configured channels.
 *
 * Stops the selected counters and restores PWM-related registers to their
 * default state.
 *
 * @param[in] instance  MPWM instance index (0-based).
 * @param[in] config    Pointer to the PWM configuration used during init.
 * @return Execution status.
 * @retval STATUS_SUCCESS  PWM mode de-initialized successfully.
 */
status_t MPWM_DRV_DeinitPwm(uint32_t instance, const mpwm_pwm_config_t *config);

/*! @} */

/*******************************************************************************
 * Runtime Update
 ******************************************************************************/
/*!
 * @name Runtime Update
 * @brief Functions for modifying PWM parameters while the channel is running.
 * @{
 */

/*!
 * @brief Update the period of a PWM channel.
 *
 * When `unit` is `MPWM_PERIOD_IN_HZ`, the function converts the target
 * frequency to period ticks before programming the hardware.
 *
 * @param[in] instance   MPWM instance index (0-based).
 * @param[in] channel    Channel index.
 * @param[in] unit       Period input unit.
 * @param[in] frequency  New frequency or period value.
 * @return Execution status.
 * @retval STATUS_SUCCESS         Period updated successfully.
 * @retval STATUS_InvalidArgument Converted period exceeds 16 bits.
 */
status_t MPWM_DRV_UpdateChannelPeriod(uint32_t instance,
                                      uint8_t channel,
                                      mpwm_period_unit_t unit,
                                      uint32_t frequency);

/*!
 * @brief Calculate the compare value for a PWM channel.
 *
 * The result depends on the configured alignment mode and update unit stored
 * in the driver state.
 *
 * @param[in] instance   MPWM instance index (0-based).
 * @param[in] channel    Channel index.
 * @param[in] period     Period value in ticks.
 * @param[in] dutyCycle  Duty-cycle value in fixed-point or ticks.
 * @return Calculated compare value in ticks.
 */
uint16_t MPWM_DRV_CalculateCompareValue(uint32_t instance,
                                        uint8_t channel,
                                        uint16_t period,
                                        uint16_t dutyCycle);

/*!
 * @brief Update the duty cycle of a PWM channel.
 *
 * @param[in] instance   MPWM instance index (0-based).
 * @param[in] channel    Channel index.
 * @param[in] dutyCycle  New duty-cycle value.
 * @return Execution status.
 * @retval STATUS_SUCCESS  Duty cycle updated successfully.
 * @retval STATUS_ERROR    The requested duty cycle is invalid for the channel.
 */
status_t MPWM_DRV_UpdateChannelDutyCycle(uint32_t instance,
                                         uint8_t channel,
                                         uint16_t dutyCycle);

/*!
 * @brief Update both the period and duty cycle of a PWM channel.
 *
 * @param[in] instance   MPWM instance index (0-based).
 * @param[in] channel    Channel index.
 * @param[in] unit       Period input unit.
 * @param[in] frequency  New frequency or period value.
 * @param[in] dutyCycle  New duty-cycle value.
 * @return Execution status.
 * @retval STATUS_SUCCESS         Update completed successfully.
 * @retval STATUS_InvalidArgument Converted period exceeds 16 bits.
 * @retval STATUS_ERROR           The requested duty cycle is invalid.
 */
status_t MPWM_DRV_UpdateChannelPeriodAndDutyCycle(uint32_t instance,
                                                  uint8_t channel,
                                                  mpwm_period_unit_t unit,
                                                  uint32_t frequency,
                                                  uint16_t dutyCycle);

/*! @} */

/*! @} */

#endif /* MPWM_PWM_DRIVER_H */
