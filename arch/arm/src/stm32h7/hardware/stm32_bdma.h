/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_bdma.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_BDMA_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_BDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_BDMA_ISR_OFFSET          0x0000 /* BDMA interrupt status register */
#define STM32_BDMA_IFCR_OFFSET         0x0004 /* BDMA interrupt flag clear register */

#define STM32_BDMACH_CCR_OFFSET        0x0008 /* BDMA channel x configuration register */
#define STM32_BDMACH_CNDTR_OFFSET      0x000c /* BDMA channel x number of data to transfer register */
#define STM32_BDMACH_CPAR_OFFSET       0x0010 /* BDMA channel x peripheral address register */
#define STM32_BDMACH_CM0AR_OFFSET      0x0014 /* BDMA channel x memory 0 address register */
#define STM32_BDMACH_CM1AR_OFFSET      0x0018 /* BDMA channel x memory 1 address register */

#define STM32_BDMA_SPACING             0x14
#define STM32_BDMA_OFFSET(x)           (STM32_BDMA_SPACING * (x))

/* Register Bitfield Definitions ********************************************/

#define BDMA_CHAN_SHIFT(n)             ((n) << 2)
#define BDMA_CHAN_MASK                 0xf
#define BDMA_CHAN_CGIF                 (1 << 0) /* Bit 0: Global interrupt flag */
#define BDMA_CHAN_TCIF                 (1 << 1) /* Bit 1: Transfer complete flag */
#define BDMA_CHAN_HTIF                 (1 << 2) /* Bit 2: Half transfer complete flag */
#define BDMA_CHAN_TEIF                 (1 << 3) /* Bit 3: Transfer error flag */
#define BDMA_CCR_ALLINTS               (BDMA_CHAN_TCIF | BDMA_CHAN_HTIF | BDMA_CHAN_TEIF)

/* BDMA channel x configuration register */

#define BDMA_CCR_EN                    (1 << 0)                    /* Bit 0: Channel enable */
#define BDMA_CCR_TCIE                  (1 << 1)                    /* Bit 1: Transfer complete interrupt enable */
#define BDMA_CCR_HTIE                  (1 << 2)                    /* Bit 2: Half Transfer interrupt enable */
#define BDMA_CCR_TEIE                  (1 << 3)                    /* Bit 3: Transfer error interrupt enable */
#define BDMA_CCR_DIR                   (1 << 4)                    /* Bit 4: Data transfer direction */
#define BDMA_CCR_CIRC                  (1 << 5)                    /* Bit 5: Circular mode */
#define BDMA_CCR_PINC                  (1 << 6)                    /* Bit 6: Peripheral increment mode */
#define BDMA_CCR_MINC                  (1 << 7)                    /* Bit 7: Memory increment */
#define BDMA_CCR_PSIZE_SHIFT           (8)                         /* Bits 8-9: Peripheral size */
#define BDMA_CCR_PSIZE_MASK            (3 << BDMA_CCR_PSIZE_SHIFT)
#  define BDMA_CCR_PSIZE_8BITS         (0 << BDMA_CCR_PSIZE_SHIFT) /* 00: 8-bits */
#  define BDMA_CCR_PSIZE_16BITS        (1 << BDMA_CCR_PSIZE_SHIFT) /* 01: 16-bits */
#  define BDMA_CCR_PSIZE_32BITS        (2 << BDMA_CCR_PSIZE_SHIFT) /* 10: 32-bits */
#define BDMA_CCR_MSIZE_SHIFT           (10)                        /* Bits 10-11: Memory size*/
#define BDMA_CCR_MSIZE_MASK            (3 << BDMA_CCR_MSIZE_SHIFT)
#  define BDMA_CCR_MSIZE_8BITS         (0 << BDMA_CCR_MSIZE_SHIFT) /* 00: 8-bits */
#  define BDMA_CCR_MSIZE_16BITS        (1 << BDMA_CCR_MSIZE_SHIFT) /* 01: 16-bits */
#  define BDMA_CCR_MSIZE_32BITS        (2 << BDMA_CCR_MSIZE_SHIFT) /* 10: 32-bits */
#define BDMA_CCR_PL_SHIFT              (12)                        /* Bits 12-13: Priority level */
#define BDMA_CCR_PL_MASK               (3 << BDMA_CCR_PL_SHIFT)
#  define BDMA_CCR_PRILO               (0 << BDMA_CCR_PL_SHIFT)    /* 00: Low */
#  define BDMA_CCR_PRIMED              (1 << BDMA_CCR_PL_SHIFT)    /* 01: Medium */
#  define BDMA_CCR_PRIHI               (2 << BDMA_CCR_PL_SHIFT)    /* 10: High */
#  define BDMA_CCR_PRIVERYHI           (3 << BDMA_CCR_PL_SHIFT)    /* 11: Very high */
#define BDMA_CCR_M2M                   (1 << 14)                   /* Bit 14: Memory-to-memory mode */
#define BDMA_CCR_DBM                   (1 << 15)                   /* Bit 15: double buffer mode */
#define BDMA_CCR_CT                    (1 << 16)                   /* Bit 16: Current target */

/* BDMA channel x number of data to transfer register */

#define BDMA_CNDTR_NDT_SHIFT           (0)       /* Bits 15-0: Number of data to Transfer */
#define BDMA_CNDTR_NDT_MASK            (0xffff << BDMA_CNDTR_NDT_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_BDMA_H */
