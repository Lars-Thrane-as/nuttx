/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sai.h
 *
 *   Copyright (C) 2023 Lars Thrane A/S. All rights reserved.
 *   Authors: Nikolaj Due Østerbye <ndo@thrane.eu>
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (c) 2016 Motorola Mobility, LLC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_STM32H7_SAI_H
#define __ARCH_ARM_SRC_STM32H7_STM32H7_SAI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/audio/i2s.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* SAI interface number (identifying the "logical" SAI interface) */

#define SAI1_BLOCK_A     0
#define SAI1_BLOCK_B     1
#define SAI4_BLOCK_A     6
#define SAI4_BLOCK_B     7

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Audio data format
 * Please read the documentation for the device connected to the other end of
 * the SAI interface. It most likely has restrictions on the format.
 *
 * Note that the SAI interface is very flexible. If we wanted to give options
 * for every configuration possible, it would be easier to just replicate
 * the actual registers. This driver implements basic TDM and I2S formats.
 *
 * (nslots != 0): TDM format is assumed.
 *                Frame length will be at least (slotsize * nslots).
 *                If nodiv is 0, then frame length is rounded up to nearest
 *                power of 2.
 * (nslots == 0): I2S format is assumed.
 *                Frame length will be (datasize * 2).
 *
 * Slot configuration values are ignored for I2S mode.
 */

struct sai_format_s
{
  uint8_t datasize:6;   /* 8, 10, 16, 20, 24, 32 bits.
                         * DMA only supports 8, 16, and 32 bits.
                         */
  uint8_t nslots:5;     /* Number of slots.
                         * TDM: 1-16
                         * I2S: 0
                         */

  /* Clock configuration must match selected data size, slot size, etc. */

  uint8_t mcken:1;
  uint8_t osr:1;
  uint8_t nodiv:1;
  uint8_t mckdiv:6;

  /* Slot configuration. Only applicable for TDM. */

  uint8_t slotsize:6;     /* 16, 32 or datasize. slotsize >= datasize. */
  uint8_t leftslot:5;     /* Index 1-16. Also used for mono. */
  uint8_t rightslot:5;    /* Index 1-16 if stereo. 0 if mono. */

};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if defined(CONFIG_STM32H7_SAI_DRIVER_SIMPLEX)
#error SAI simplex driver not imlpemented

/****************************************************************************
 * Name: stm32_sai_initialize
 *
 * Description:
 *   Initialize the selected SAI interface
 *
 * Input Parameters:
 *   intf   - SAI interface number (identifying the "logical" SAI interface)
 *   format - Pointer to static struct that describes the audio format
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *stm32_sai_initialize(int intf, struct sai_format_s *format);

#endif /* CONFIG_STM32H7_SAI_DRIVER_SIMPLEX */

#if defined(CONFIG_STM32H7_SAI_DRIVER_DUPLEX)

#if (!defined(CONFIG_STM32H7_SAI4_A_SYNC_WITH_B) && \
     !defined(CONFIG_STM32H7_SAI4_B_SYNC_WITH_A) && \
     !defined(CONFIG_STM32H7_SAI1_A_SYNC_WITH_B) && \
     !defined(CONFIG_STM32H7_SAI1_B_SYNC_WITH_A))
#  error SAI full duplex mode requires two SAI interfaces in sync
#endif


/****************************************************************************
 * Name: stm32_sai_duplex_initialize
 *
 * Description:
 *   Initialize both SAI blocks in a synchronous setup.
 *   RX and TX interfaces must belong to the same SAI (SAI1 or SAI4).
 *
 * Input Parameters:
 *   block_rx - SAI block id for RX interface
 *   block_tx - SAI block id for TX interface
 *   format   - Pointer to static struct that describes the audio format
 *
 * Returned Value:
 *   Valid Master I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *stm32_sai_duplex_initialize(int block_rx, int block_tx,
                                              struct sai_format_s *format);
#endif /* CONFIG_STM32H7_SAI_DRIVER_DUPLEX */

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32H7_STM32H7_SAI_H */
