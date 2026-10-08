
/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * @file vector.S
 * @brief 
 * 
 */

    

    SECTION .isr_vector:CODE:ROOT(2)
    EXTERN DefaultISR
    EXTERN STACK_end   /* Top of Stack for Initial Stack Pointer */
    EXTERN Reset_Handler             /* Reset Handler */
    EXTERN   NMI_Handler
    EXTERN   HardFault_Handler
    EXTERN   SVC_Handler
    EXTERN   PendSV_Handler
    EXTERN   SysTick_Handler
    EXTERN   DMA0_IRQHandler
    EXTERN   DMA1_IRQHandler
    EXTERN   DMA2_IRQHandler
    EXTERN   DMA3_IRQHandler
    EXTERN   DMA_Error_IRQHandler
    EXTERN   EMU0_IRQHandler
    EXTERN   MPWM0_Ovf_IRQHandler
    EXTERN   MPWM0_Ch0_Ch7_IRQHandler
    EXTERN   lpTMR0_IRQHandler
    EXTERN   GPIOA_IRQHandler
    EXTERN   CAN0_ORed_Err_Wakeup_IRQHandler
    EXTERN   CAN0_ORed_0_15_MB_IRQHandler
    EXTERN   eTMR0_CH_0_7_IRQHandler
    EXTERN   eTMR0_Fault_IRQHandler
    EXTERN   eTMR0_Ovf_IRQHandler
    EXTERN   GPIOB_IRQHandler
    EXTERN   GPIOC_IRQHandler
    EXTERN   CAN0_ORed_16_31_MB_IRQHandler
    EXTERN   EFM_IRQHandler
    EXTERN   GPIOD_IRQHandler
    EXTERN   pTMR0_IRQHandler
    EXTERN   PCU_CMU_IRQHandler
    EXTERN   WDG0_IRQHandler
    EXTERN   GPIOE_IRQHandler
    EXTERN   I2C0_IRQHandler
    EXTERN   SPI0_IRQHandler
    EXTERN   SPI1_IRQHandler
    EXTERN   ADC0_IRQHandler
    EXTERN   ACMP0_IRQHandler
    EXTERN   UART1_IRQHandler
    EXTERN   UART0_IRQHandler

    DATA
__vector_table
    DCD STACK_end   /* Top of Stack for Initial Stack Pointer */
    DCD Reset_Handler             /* Reset Handler */
    DCD NMI_Handler
    DCD HardFault_Handler
    DCD DefaultISR
    DCD DefaultISR
    DCD DefaultISR
    DCD DefaultISR
    DCD DefaultISR
    DCD DefaultISR
    DCD DefaultISR
    DCD SVC_Handler
    DCD DefaultISR
    DCD DefaultISR
    DCD PendSV_Handler
    DCD SysTick_Handler
    DCD DMA0_IRQHandler
    DCD DMA1_IRQHandler
    DCD DMA2_IRQHandler
    DCD DMA3_IRQHandler
    DCD DMA_Error_IRQHandler
    DCD EMU0_IRQHandler
    DCD MPWM0_Ovf_IRQHandler
    DCD MPWM0_Ch0_Ch7_IRQHandler
    DCD lpTMR0_IRQHandler
    DCD GPIOA_IRQHandler
    DCD CAN0_ORed_Err_Wakeup_IRQHandler
    DCD CAN0_ORed_0_15_MB_IRQHandler
    DCD eTMR0_CH_0_7_IRQHandler
    DCD eTMR0_Fault_IRQHandler
    DCD eTMR0_Ovf_IRQHandler
    DCD GPIOB_IRQHandler
    DCD GPIOC_IRQHandler
    DCD CAN0_ORed_16_31_MB_IRQHandler
    DCD EFM_IRQHandler
    DCD GPIOD_IRQHandler
    DCD pTMR0_IRQHandler
    DCD PCU_CMU_IRQHandler
    DCD WDG0_IRQHandler
    DCD GPIOE_IRQHandler
    DCD I2C0_IRQHandler
    DCD DefaultISR
    DCD SPI0_IRQHandler
    DCD SPI1_IRQHandler
    DCD ADC0_IRQHandler
    DCD ACMP0_IRQHandler
    DCD UART1_IRQHandler
    DCD UART0_IRQHandler
    END
