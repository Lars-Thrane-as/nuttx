/****************************************************************************
 * arch/arm/include/stm32h7/stm32h72x73x_irq.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* This file should never be included directly but, rather,
 * only indirectly through arch/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32H7_STM32H72X73X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to bits
 * in the NVIC.
 * This does, however, waste several words of memory in the IRQ to handle
 * mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be
 * found in the file nuttx/arch/arm/include/stm32h7/irq.h which includes
 * this file
 *
 * External interrupts (vectors >= 16)
 */

#define STM32_IRQ_WWDG1        (STM32_IRQ_FIRST + 0)    /*  0: Window Watchdog interrupt */
#define STM32_IRQ_PVDPVM       (STM32_IRQ_FIRST + 1)    /*  1: PVD through EXTI line detection interrupt */
#define STM32_IRQ_RTC          (STM32_IRQ_FIRST + 2)    /*  2: RTC tamper, timestamp */
#define STM32_IRQ_CSSLSE       (STM32_IRQ_FIRST + 2)    /*  2: CSS LSE */
#define STM32_IRQ_RTCWKUP      (STM32_IRQ_FIRST + 3)    /*  3: RTC Wakeup interrupt through the EXTI line */
#define STM32_IRQ_FLASH        (STM32_IRQ_FIRST + 4)    /*  4: Flash memory global interrupt */
#define STM32_IRQ_RCC          (STM32_IRQ_FIRST + 5)    /*  5: RCC global interrupt */
#define STM32_IRQ_EXTI0        (STM32_IRQ_FIRST + 6)    /*  6: EXTI Line 0 interrupt */
#define STM32_IRQ_EXTI1        (STM32_IRQ_FIRST + 7)    /*  7: EXTI Line 1 interrupt */
#define STM32_IRQ_EXTI2        (STM32_IRQ_FIRST + 8)    /*  8: EXTI Line 2 interrupt */
#define STM32_IRQ_EXTI3        (STM32_IRQ_FIRST + 9)    /*  9: EXTI Line 3 interrupt */
#define STM32_IRQ_EXTI4        (STM32_IRQ_FIRST + 10)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_DMA1S0       (STM32_IRQ_FIRST + 11)   /* 11: DMA1 Stream0 global interrupt */
#define STM32_IRQ_DMA1S1       (STM32_IRQ_FIRST + 12)   /* 12: DMA1 Stream1 global interrupt */
#define STM32_IRQ_DMA1S2       (STM32_IRQ_FIRST + 13)   /* 13: DMA1 Stream2 global interrupt */
#define STM32_IRQ_DMA1S3       (STM32_IRQ_FIRST + 14)   /* 14: DMA1 Stream3 global interrupt */
#define STM32_IRQ_DMA1S4       (STM32_IRQ_FIRST + 15)   /* 15: DMA1 Stream4 global interrupt */
#define STM32_IRQ_DMA1S5       (STM32_IRQ_FIRST + 16)   /* 16: DMA1 Stream5 global interrupt */
#define STM32_IRQ_DMA1S6       (STM32_IRQ_FIRST + 17)   /* 17: DMA1 Stream6 global interrupt */
#define STM32_IRQ_ADC12        (STM32_IRQ_FIRST + 18)   /* 18: ADC1 and ADC2 global interrupt */
#define STM32_IRQ_FDCAN1_0     (STM32_IRQ_FIRST + 19)   /* 19: FDCAN1 Interrupt 0 */
#define STM32_IRQ_FDCAN2_0     (STM32_IRQ_FIRST + 20)   /* 20: FDCAN2 Interrupt 0 */
#define STM32_IRQ_FDCAN1_1     (STM32_IRQ_FIRST + 21)   /* 21: FDCAN1 Interrupt 1 */
#define STM32_IRQ_FDCAN2_1     (STM32_IRQ_FIRST + 22)   /* 22: FDCAN2 Interrupt 1 */
#define STM32_IRQ_EXTI95       (STM32_IRQ_FIRST + 23)   /* 23: EXTI Line[9:5] interrupts */
#define STM32_IRQ_TIM1BRK      (STM32_IRQ_FIRST + 24)   /* 24: TIM1 break interrupt */
#define STM32_IRQ_TIM1UP       (STM32_IRQ_FIRST + 25)   /* 25: TIM1 update interrupt */
#define STM32_IRQ_TIM1TRGCOM   (STM32_IRQ_FIRST + 26)   /* 26: TIM1 trigger and commutation interrupts */
#define STM32_IRQ_TIMCC        (STM32_IRQ_FIRST + 27)   /* 27: TIM1 capture / compare interrupt */
#define STM32_IRQ_TIM2         (STM32_IRQ_FIRST + 28)   /* 28: TIM2 global interrupt */
#define STM32_IRQ_TIM3         (STM32_IRQ_FIRST + 29)   /* 29: TIM3 global interrupt */
#define STM32_IRQ_TIM4         (STM32_IRQ_FIRST + 30)   /* 30: TIM4 global interrupt */
#define STM32_IRQ_I2C1EV       (STM32_IRQ_FIRST + 31)   /* 31: I2C1 event interrupt */
#define STM32_IRQ_I2C1ER       (STM32_IRQ_FIRST + 32)   /* 32: I2C1 error interrupt */
#define STM32_IRQ_I2C2EV       (STM32_IRQ_FIRST + 33)   /* 33: I2C2 event interrupt */
#define STM32_IRQ_I2C2ER       (STM32_IRQ_FIRST + 34)   /* 34: I2C2 error interrupt */
#define STM32_IRQ_SPI1         (STM32_IRQ_FIRST + 35)   /* 35: SPI1 global interrupt */
#define STM32_IRQ_SPI2         (STM32_IRQ_FIRST + 36)   /* 36: SPI2 global interrupt */
#define STM32_IRQ_USART1       (STM32_IRQ_FIRST + 37)   /* 37: USART1 global interrupt */
#define STM32_IRQ_USART2       (STM32_IRQ_FIRST + 38)   /* 38: USART2 global interrupt */
#define STM32_IRQ_USART3       (STM32_IRQ_FIRST + 39)   /* 39: USART3 global interrupt */
#define STM32_IRQ_EXTI1510     (STM32_IRQ_FIRST + 40)   /* 40: EXTI Line[15:10] interrupts */
#define STM32_IRQ_RTCALARM     (STM32_IRQ_FIRST + 41)   /* 41: RTC alarms (A and B) through EXTI Line interrupts */
#define STM32_IRQ_RESERVED42   (STM32_IRQ_FIRST + 42)   /* 42: Reserved */
#define STM32_IRQ_TIM8BRK      (STM32_IRQ_FIRST + 43)   /* 43: TIM8 break interrupt */
#define STM32_IRQ_TIM12        (STM32_IRQ_FIRST + 43)   /* 43: TIM12 global interrupt */
#define STM32_IRQ_TIM8UP       (STM32_IRQ_FIRST + 44)   /* 44: TIM8 update interrupt */
#define STM32_IRQ_TIM13        (STM32_IRQ_FIRST + 44)   /* 44: TIM13 global interrupt */
#define STM32_IRQ_TIM8TRGCOM   (STM32_IRQ_FIRST + 45)   /* 45: TIM8 trigger /commutation interrupt */
#define STM32_IRQ_TIM14        (STM32_IRQ_FIRST + 45)   /* 45: TIM14 global interrupts */
#define STM32_IRQ_TIM8CC       (STM32_IRQ_FIRST + 46)   /* 46: TIM8 capture / compare interrupts */
#define STM32_IRQ_DMA1S7       (STM32_IRQ_FIRST + 47)   /* 47: DMA1 Stream7 global interrupt */
#define STM32_IRQ_FMC          (STM32_IRQ_FIRST + 48)   /* 48: FMC global interrupt */
#define STM32_IRQ_SDMMC1       (STM32_IRQ_FIRST + 49)   /* 49: SDMMC1 global interrupt */
#define STM32_IRQ_TIM5         (STM32_IRQ_FIRST + 50)   /* 50: TIM5 global interrupt */
#define STM32_IRQ_SPI3         (STM32_IRQ_FIRST + 51)   /* 51: SPI3 global interrupt */
#define STM32_IRQ_UART4        (STM32_IRQ_FIRST + 52)   /* 52: UART4 global interrupt */
#define STM32_IRQ_UART5        (STM32_IRQ_FIRST + 53)   /* 53: UART5 global interrupt */
#define STM32_IRQ_TIM6         (STM32_IRQ_FIRST + 54)   /* 54: TIM6 global interrupt */
#define STM32_IRQ_DAC1         (STM32_IRQ_FIRST + 54)   /* 54: DAC1 underrun error interrupt */
#define STM32_IRQ_DAC2         (STM32_IRQ_FIRST + 54)   /* 54: DAC2 underrun error interrupt */
#define STM32_IRQ_TIM7         (STM32_IRQ_FIRST + 55)   /* 55: TIM7 global interrupt */
#define STM32_IRQ_DMA2S0       (STM32_IRQ_FIRST + 56)   /* 56: DMA2 Stream0 interrupt */
#define STM32_IRQ_DMA2S1       (STM32_IRQ_FIRST + 57)   /* 57: DMA2 Stream1 interrupt */
#define STM32_IRQ_DMA2S2       (STM32_IRQ_FIRST + 58)   /* 58: FMA2 Stream2 interrupt */
#define STM32_IRQ_DMA2S3       (STM32_IRQ_FIRST + 59)   /* 59: DMA2 Stream3 interrupt */
#define STM32_IRQ_DMA2S4       (STM32_IRQ_FIRST + 60)   /* 60: DMA2 Stream4 interrupt */
#define STM32_IRQ_ETH          (STM32_IRQ_FIRST + 61)   /* 61: Ethernet global interrupt */
#define STM32_IRQ_ETHWKUP      (STM32_IRQ_FIRST + 62)   /* 62: Ethernet wakeup through EXTI line interrupt */
#define STM32_IRQ_FDCANCAL     (STM32_IRQ_FIRST + 63)   /* 63: CAN2TX interrupts */
#define STM32_IRQ_RESERVED64   (STM32_IRQ_FIRST + 64)   /* 64: Reserved */
#define STM32_IRQ_RESERVED65   (STM32_IRQ_FIRST + 65)   /* 65: Reserved */
#define STM32_IRQ_RESERVED66   (STM32_IRQ_FIRST + 66)   /* 66: Reserved */
#define STM32_IRQ_RESERVED67   (STM32_IRQ_FIRST + 67)   /* 67: Reserved */
#define STM32_IRQ_DMA2S5       (STM32_IRQ_FIRST + 68)   /* 68: DMA2 Stream5 interrupt */
#define STM32_IRQ_DMA2S6       (STM32_IRQ_FIRST + 69)   /* 69: DMA2 Stream6 interrupt */
#define STM32_IRQ_DMA2S7       (STM32_IRQ_FIRST + 70)   /* 70: DMA2 Stream7 interrupt */
#define STM32_IRQ_USART6       (STM32_IRQ_FIRST + 71)   /* 71: USART6 global interrupt */
#define STM32_IRQ_I2C3EV       (STM32_IRQ_FIRST + 72)   /* 72: I2C3 event interrupt */
#define STM32_IRQ_I2C3ER       (STM32_IRQ_FIRST + 73)   /* 73: I2C3 error interrupt*/
#define STM32_IRQ_OTGHS_EP1OUT (STM32_IRQ_FIRST + 74)   /* 74: OTG_HS out global interrupt */
#define STM32_IRQ_OTGHS_EP1IN  (STM32_IRQ_FIRST + 75)   /* 75: OTG_HS in global interrupt */
#define STM32_IRQ_OTGHS_WKUP   (STM32_IRQ_FIRST + 76)   /* 76: OTG_HS wakeup interrupt */
#define STM32_IRQ_OTGHS        (STM32_IRQ_FIRST + 77)   /* 77: OTG_HS global interrupt */
#define STM32_IRQ_DCMI         (STM32_IRQ_FIRST + 78)   /* 78: DCMI global interrupt */
#define STM32_IRQ_CRYP         (STM32_IRQ_FIRST + 79)   /* 79: CRYP global interrupt */
#define STM32_IRQ_HASH         (STM32_IRQ_FIRST + 80)   /* 80: HASH global interrupt */
#define STM32_IRQ_RNG          (STM32_IRQ_FIRST + 80)   /* 80: RNG global interrupt */
#define STM32_IRQ_FPU          (STM32_IRQ_FIRST + 81)   /* 81: CPU FPU */
#define STM32_IRQ_UART7        (STM32_IRQ_FIRST + 82)   /* 82: UART7 global interrupt */
#define STM32_IRQ_UART8        (STM32_IRQ_FIRST + 83)   /* 83: UART8 global interrupt */
#define STM32_IRQ_SPI4         (STM32_IRQ_FIRST + 84)   /* 84: SPI4 global interrupt */
#define STM32_IRQ_SPI5         (STM32_IRQ_FIRST + 85)   /* 85: SPI5 global interrupt */
#define STM32_IRQ_SPI6         (STM32_IRQ_FIRST + 86)   /* 86: SPI6  global interrupt */
#define STM32_IRQ_SAI1         (STM32_IRQ_FIRST + 87)   /* 87: SAI1  global interrupt */
#define STM32_IRQ_LTDC         (STM32_IRQ_FIRST + 88)   /* 88: LCD-TFT global interrupt */
#define STM32_IRQ_LTDCER       (STM32_IRQ_FIRST + 89)   /* 89: LCD-TFT error interrupt */
#define STM32_IRQ_DMA2D        (STM32_IRQ_FIRST + 90)   /* 90: DMA2D global interrupt */
#define STM32_IRQ_RESERVED91   (STM32_IRQ_FIRST + 91)   /* 91: Reserved */
#define STM32_IRQ_OCTOSPI1     (STM32_IRQ_FIRST + 92)   /* 92: OCTOSPI1 global interrupt */
#define STM32_IRQ_LPTIM1       (STM32_IRQ_FIRST + 93)   /* 93: LPTIM1 global interrupt */
#define STM32_IRQ_CEC          (STM32_IRQ_FIRST + 94)   /* 94: HDMI-CEC global interrupt */
#define STM32_IRQ_I2C4EV       (STM32_IRQ_FIRST + 95)   /* 95: I2C4 event interrupt */
#define STM32_IRQ_I2C4ER       (STM32_IRQ_FIRST + 96)   /* 96: I2C4 error interrupt */
#define STM32_IRQ_SPDIF        (STM32_IRQ_FIRST + 97)   /* 97: SPDIFRX global interrupt */
#define STM32_IRQ_RESERVED98   (STM32_IRQ_FIRST + 98)   /* 98: Reserved */
#define STM32_IRQ_RESERVED99   (STM32_IRQ_FIRST + 99)   /* 99: Reserved */
#define STM32_IRQ_RESERVED100  (STM32_IRQ_FIRST + 100)  /* 100: Reserved */
#define STM32_IRQ_RESERVED101  (STM32_IRQ_FIRST + 101)  /* 101: Reserved */
#define STM32_IRQ_DMAMUX1OV    (STM32_IRQ_FIRST + 102)  /* 102: DMAMUX1 overrun interrupt */
#define STM32_IRQ_RESERVED103  (STM32_IRQ_FIRST + 103)  /* 103: Reserved */
#define STM32_IRQ_RESERVED104  (STM32_IRQ_FIRST + 104)  /* 104: Reserved */
#define STM32_IRQ_RESERVED105  (STM32_IRQ_FIRST + 105)  /* 105: Reserved */
#define STM32_IRQ_RESERVED106  (STM32_IRQ_FIRST + 106)  /* 106: Reserved */
#define STM32_IRQ_RESERVED107  (STM32_IRQ_FIRST + 107)  /* 107: Reserved */
#define STM32_IRQ_RESERVED108  (STM32_IRQ_FIRST + 108)  /* 108: Reserved */
#define STM32_IRQ_RESERVED109  (STM32_IRQ_FIRST + 109)  /* 109: Reserved */
#define STM32_IRQ_DFSDM1FLT0   (STM32_IRQ_FIRST + 110)  /* 110: DFSDM1 filter 0 interrupt */
#define STM32_IRQ_DFSDM1FLT1   (STM32_IRQ_FIRST + 111)  /* 111: DFSDM1 filter 1 interrupt */
#define STM32_IRQ_DFSDM1FLT2   (STM32_IRQ_FIRST + 112)  /* 112: DFSDM1 filter 2 interrupt */
#define STM32_IRQ_DFSDM1FLT3   (STM32_IRQ_FIRST + 113)  /* 113: DFSDM1 filter 3 interrupt */
#define STM32_IRQ_RESERVED114  (STM32_IRQ_FIRST + 114)  /* 114: Reserved */
#define STM32_IRQ_SWPMI1       (STM32_IRQ_FIRST + 115)  /* 115: SWPMI global interrupt/wakeup */
#define STM32_IRQ_TIM15        (STM32_IRQ_FIRST + 116)  /* 116: TIM15 global interrupt */
#define STM32_IRQ_TIM16        (STM32_IRQ_FIRST + 117)  /* 117: TIM16 global interrupt */
#define STM32_IRQ_TIM17        (STM32_IRQ_FIRST + 118)  /* 118: TIM17 global interrupt */
#define STM32_IRQ_MDIOSWKUP    (STM32_IRQ_FIRST + 119)  /* 119: MDIOS wakeup */
#define STM32_IRQ_MDIOS        (STM32_IRQ_FIRST + 120)  /* 120: MDIOS global interrupt */
#define STM32_IRQ_RESERVED121  (STM32_IRQ_FIRST + 121)  /* 121: Reserved */
#define STM32_IRQ_MDMA         (STM32_IRQ_FIRST + 122)  /* 122: MDMA global interrupt */
#define STM32_IRQ_RESERVED123  (STM32_IRQ_FIRST + 123)  /* 123: Reserved */
#define STM32_IRQ_SDMMC2       (STM32_IRQ_FIRST + 124)  /* 124: SDMMC2 global interrupt */
#define STM32_IRQ_HSEM0        (STM32_IRQ_FIRST + 125)  /* 125: HSEM global interrupt 1 */
#define STM32_IRQ_RESERVED126  (STM32_IRQ_FIRST + 126)  /* 126: Reserved */
#define STM32_IRQ_ADC3         (STM32_IRQ_FIRST + 127)  /* 127: ADC3 global interrupt */
#define STM32_IRQ_DMAMUX2OVR   (STM32_IRQ_FIRST + 128)  /* 128: DMAMUX2 overrun interrupt */
#define STM32_IRQ_BDMACH1      (STM32_IRQ_FIRST + 129)  /* 129: BDMA channel 1 interrupt */
#define STM32_IRQ_BDMACH2      (STM32_IRQ_FIRST + 130)  /* 130: BDMA channel 2 interrupt */
#define STM32_IRQ_BDMACH3      (STM32_IRQ_FIRST + 131)  /* 131: BDMA channel 3 interrupt */
#define STM32_IRQ_BDMACH4      (STM32_IRQ_FIRST + 132)  /* 132: BDMA channel 4 interrupt */
#define STM32_IRQ_BDMACH5      (STM32_IRQ_FIRST + 133)  /* 133: BDMA channel 5 interrupt */
#define STM32_IRQ_BDMACH6      (STM32_IRQ_FIRST + 134)  /* 134: BDMA channel 6 interrupt */
#define STM32_IRQ_BDMACH7      (STM32_IRQ_FIRST + 135)  /* 135: BDMA channel 7 interrupt */
#define STM32_IRQ_BDMACH8      (STM32_IRQ_FIRST + 136)  /* 136: BDMA channel 8 interrupt */
#define STM32_IRQ_COMP         (STM32_IRQ_FIRST + 137)  /* 137: COMP1 and COMP2 exti_comp1_wkup global interrupt */
#define STM32_IRQ_LPTIM2       (STM32_IRQ_FIRST + 138)  /* 138: LPTIM2 timer interrupt */
#define STM32_IRQ_LPTIM3       (STM32_IRQ_FIRST + 139)  /* 139: LPTIM3 timer interrupt */
#define STM32_IRQ_LPTIM4       (STM32_IRQ_FIRST + 140)  /* 140: LPTIM4 timer interrupt */
#define STM32_IRQ_LPTIM5       (STM32_IRQ_FIRST + 141)  /* 141: LPTIM5 timer interrupt */
#define STM32_IRQ_LPUART       (STM32_IRQ_FIRST + 142)  /* 142: LPUART global interrupt */
#define STM32_IRQ_RESERVED143  (STM32_IRQ_FIRST + 143)  /* 143: Reserved */
#define STM32_IRQ_CRS          (STM32_IRQ_FIRST + 144)  /* 144: Clock Recovery System global interrupt */
#define STM32_IRQ_ECCDIAG      (STM32_IRQ_FIRST + 145)  /* 145: ECC diagnostics global interrupt */
#define STM32_IRQ_SAI4         (STM32_IRQ_FIRST + 146)  /* 146: SAI4  global interrupt */
#define STM32_IRQ_TEMP         (STM32_IRQ_FIRST + 147)  /* 147: Temperature sensor global interrupt */
#define STM32_IRQ_RESERVED148  (STM32_IRQ_FIRST + 148)  /* 148: Reserved */
#define STM32_IRQ_WKUP         (STM32_IRQ_FIRST + 149)  /* 149: WKUP1 to WKUP6 pins */
#define STM32_IRQ_OCTOSPI2     (STM32_IRQ_FIRST + 150)  /* 150: OCTOSPI2 global interrupt */
#define STM32_IRQ_OTFDEC1      (STM32_IRQ_FIRST + 151)  /* 151: OTFDEC1 interrupt */
#define STM32_IRQ_OTFDEC2      (STM32_IRQ_FIRST + 152)  /* 152: OTFDEC2 interrupt */
#define STM32_IRQ_FMAC         (STM32_IRQ_FIRST + 153)  /* 153: FMAC intterupt */
#define STM32_IRQ_CORDIC       (STM32_IRQ_FIRST + 154)  /* 154: CORDIC interrupt */
#define STM32_IRQ_UART9        (STM32_IRQ_FIRST + 155)  /* 155: UART9 interrupt */
#define STM32_IRQ_USART10      (STM32_IRQ_FIRST + 156)  /* 156: USART10 interrupt */
#define STM32_IRQ_I2C5EV       (STM32_IRQ_FIRST + 157)  /* 157: I2C5 event interrupt */
#define STM32_IRQ_I2C5ER       (STM32_IRQ_FIRST + 158)  /* 158: I2C5 error interrupt */
#define STM32_IRQ_FDCAN3_0     (STM32_IRQ_FIRST + 159)  /* 159: FDCAN3 interrupt 0 */
#define STM32_IRQ_FDCAN3_1     (STM32_IRQ_FIRST + 160)  /* 160: FDCAN3 interrupt 1 */
#define STM32_IRQ_TIM23        (STM32_IRQ_FIRST + 161)  /* 161: TIM23 interrupt */
#define STM32_IRQ_TIM24        (STM32_IRQ_FIRST + 162)  /* 162: TIM24 interrupt */


#define STM32_IRQ_NEXTINTS     163
#define NR_IRQS                (STM32_IRQ_FIRST + STM32_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_STM32H7_STM32H72X73X_IRQ_H */
