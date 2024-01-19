/****************************************************************************
 * arch/arm/src/stm32h7/stm32h72x73x_vrefbuf.c
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

#include "chip.h"
#include "arm_internal.h"
#include "hardware/stm32h72x73x_vrefbuf.h"

/****************************************************************************
 * Name: vrefbuf_initialize
 *
 * Description:
 *   Initialize VREFBUF.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) or error value
 * 
 ****************************************************************************/
static int vrefbuf_initialize(void)
{
  const uint32_t vrs = (CONFIG_STM32H7_VREFBUF_VREF << VREFBUF_CSR_VRS_SHIFT);

  switch(vrs)
    {
      case VREFBUF_CSR_VRS_2_5V:
      case VREFBUF_CSR_VRS_2_048V:
      case VREFBUF_CSR_VRS_1_8V:
      case VREFBUF_CSR_VRS_1_5V:
        break;
      default:
        return -EINVAL;
    }
  
  /* Set reference voltage and enable VREF buffer */
  putreg32(vrs | VREFBUF_CSR_ENVR, STM32_VREFBUF_CSR);

  /* Wait for VRR to be set */
  while((getreg32(STM32_VREFBUF_CSR) & VREFBUF_CSR_VRR) == 0);

  return OK;
}
