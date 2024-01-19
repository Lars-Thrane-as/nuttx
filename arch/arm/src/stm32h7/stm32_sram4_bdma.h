/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sram4_bdma.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_SRAM4_BDMA_H
#define __ARCH_ARM_SRC_STM32H7_STM32_SRAM4_BDMA_H

/****************************************************************************
 * The purpose of this driver is to disable data caching of the part of
 * SRAM4 that is used with the BDMA. Data caching is disabled through the
 * MPU.
 *
 * The driver supports:
 *  - SAI4
 *
 * If another peripheral needs to use the BDMA, please add the memory size
 * to the cache-disabled area.
 *
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_sram4_bdma_initialize
 *
 * Description:
 *   Disable data caching for parts of SRAM4 that is used with the BDMA.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

void stm32_sram4_bdma_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_SRAM4_BDMA_H */
