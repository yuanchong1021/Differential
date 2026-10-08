/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * @file pin_mux.c
 * @brief 
 * 
 */



#include "pin_mux.h"

/*! @brief User configuration cfg InitConfigArr0 array */
const pin_settings_config_t g_pin_mux_InitConfigArr0[NUM_OF_CONFIGURED_PINS0] = {
    /* PTA_2-17-MPWM0_CH1-PWM_B */
    {
        .base = PCTRLA,
        .pinPortIdx = 2U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_ALT2,
        .pinLock = false,
        .intConfig = PCTRL_DMA_INT_DISABLED,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOA,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 0,
    },
    /* PTA_3-16-MPWM0_CH0-PWM_A */
    {
        .base = PCTRLA,
        .pinPortIdx = 3U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_ALT2,
        .pinLock = false,
        .intConfig = PCTRL_DMA_INT_DISABLED,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOA,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 0,
    },
    /* PTE_4-4-GPIO-SWA */
    {
        .base = PCTRLE,
        .pinPortIdx = 4U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_AS_GPIO,
        .pinLock = false,
        .intConfig = PCTRL_DMA_INT_DISABLED,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOE,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 0,
    },
    /* PTE_5-3-GPIO-SWB */
    {
        .base = PCTRLE,
        .pinPortIdx = 5U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_AS_GPIO,
        .pinLock = false,
        .intConfig = PCTRL_DMA_INT_DISABLED,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOE,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 0,
    },
    /* PTB_0-13-GPIO-RX-OUT */
    {
        .base = PCTRLB,
        .pinPortIdx = 0U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_AS_GPIO,
        .pinLock = false,
        .intConfig = PCTRL_DMA_INT_DISABLED,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOB,
        .direction = GPIO_OUTPUT_DIRECTION,
        .initValue = 0,
    },
    /* PTB_1-12-GPIO-TX-OUT */
    {
        .base = PCTRLB,
        .pinPortIdx = 1U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_AS_GPIO,
        .pinLock = false,
        .intConfig = PCTRL_DMA_INT_DISABLED,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOB,
        .direction = GPIO_OUTPUT_DIRECTION,
        .initValue = 0,
    },
    /* PTA_0-19-GPIO-hall_a */
    {
        .base = PCTRLA,
        .pinPortIdx = 0U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_AS_GPIO,
        .pinLock = false,
        .intConfig = PCTRL_INT_FALLING_EDGE,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOA,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 1,
    },
    /* PTA_1-18-GPIO-hall_b */
    {
        .base = PCTRLA,
        .pinPortIdx = 1U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_AS_GPIO,
        .pinLock = false,
        .intConfig = PCTRL_INT_EITHER_EDGE,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOA,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 1,
    },
    /* PTD_3-15-GPIO-hall_c */
    {
        .base = PCTRLD,
        .pinPortIdx = 3U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_AS_GPIO,
        .pinLock = false,
        .intConfig = PCTRL_INT_FALLING_EDGE,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOD,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 1,
    },
    /* PTB_2-11-GPIO-hall_d */
    {
        .base = PCTRLB,
        .pinPortIdx = 2U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_AS_GPIO,
        .pinLock = false,
        .intConfig = PCTRL_INT_FALLING_EDGE,
        .clearIntFlag = false,
        .digitalFilter = false,
        .filterConfig ={
            .width = 0U,
        },
        .gpioBase = GPIOB,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 1,
    },
};



/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/

