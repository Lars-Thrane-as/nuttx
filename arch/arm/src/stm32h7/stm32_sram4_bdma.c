/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sram4_bdma.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <chip.h>

#include "stm32_sram4_bdma.h"
#include "mpu.h"
#include "hardware/stm32_memorymap.h"

#ifdef CONFIG_STM32H7_SRAM4_BDMA

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#  if (defined(CONFIG_STM32H7_SAI1_A) || defined(CONFIG_STM32H7_SAI1_B) || \
       defined(CONFIG_STM32H7_SAI4_A) || defined(CONFIG_STM32H7_SAI4_B))
#    define SAI_BUFFER_ADJUSTED_SIZE DCACHE_BUFFER_SIZE_ADJUST(CONFIG_STM32H7_SAI_BDMA_BUFFER_SIZE)
#  endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_sram4_bdma_initialize
 *
 * Description:
 *   The STM32H7 BDMA memory must be located in SRAM4. The linker script
 *   defines an area at the beginning of SRAM4 where memory can be statically
 *   allocated for use with BDMA.
 *   This memory must now have caching disabled. We do this by using the MPU.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_sram4_bdma_initialize(void)
{
#if defined(CONFIG_ARMV7M_DCACHE)
  size_t size = 0;

  /* SAI4 duplex driver uses double buffering, so it will allocate 2x buffer
   * size per interface.
   */

# if defined(CONFIG_STM32H7_SAI4_A)
  size += (SAI_BUFFER_ADJUSTED_SIZE * 2);
# endif
# if defined(CONFIG_STM32H7_SAI4_B)
  size += (SAI_BUFFER_ADJUSTED_SIZE * 2);
# endif

  if (size > 0)
    {
# if defined(CONFIG_BUILD_PROTECTED)
      mpu_peripheral(STM32_SRAM4_BASE, size);
# else
      mpu_user_peripheral(STM32_SRAM4_BASE, size);
      mpu_control(true, true, true);
# endif /* CONFIG_BUILD_PROTECTED */
    }
#endif /* CONFIG_ARMV7M_DCACHE */
}

#endif /* CONFIG_STM32H7_SRAM4_BDMA */
