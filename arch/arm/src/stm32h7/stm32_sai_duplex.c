/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sai_duplex.c
 *
 *   Copyright (C) 2023 Lars Thrane A/S. All rights reserved.
 *   Authors: Nikolaj Due Østerbye <ndo@thrane.eu>
 *   Copyright (C) 2013-2014, 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32H7_SAI_DRIVER_DUPLEX

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include "arm_internal.h"
#include "stm32_dma.h"
#include "stm32_gpio.h"
#include "stm32_sai.h"
#include "hardware/stm32_rcc.h"
#include "hardware/stm32_sai.h"

#if (defined(CONFIG_STM32H7_SAI1_A) || defined(CONFIG_STM32H7_SAI1_B))
#  error "Driver doesn't support SAI1 yet."
// TODO: handle clock and DMA
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S required by this driver
#endif

/* SAI1 uses standard DMA.
 * SAI4 uses BDMA.
 *
 * DMA driver will automatically translate standard DMA register values to
 * BDMA register values, if needed. So we can simply do one DMA configuration.
 */

#if (defined(CONFIG_STM32H7_SAI4_A) || defined(CONFIG_STM32H7_SAI4_B))
#  ifndef CONFIG_STM32H7_BDMA
#    error CONFIG_STM32H7_BDMA required by this driver
#  endif
#endif

#if (defined(CONFIG_STM32H7_SAI1_A) || defined(CONFIG_STM32H7_SAI1_B))
#   warning A check for DMA enablement should be done here
#endif

#if defined(CONFIG_STM32H7_SAI_DMAPRIO)
#  define SAI_DMA_PRIO       CONFIG_STM32H7_SAI_DMAPRIO
#else
#  define SAI_DMA_PRIO       DMA_SCR_PRIVERYHI
#endif

#if (SAI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#  error "Illegal value for CONFIG_STM32H7_SAI_DMAPRIO"
#endif

#define SAI_DMA_BASE_CR   (DMA_SCR_CIRC |DMA_SCR_MINC | SAI_DMA_PRIO | \
                           DMA_SCR_DBM | DMA_SCR_PBURST_INCR4 | \
                           DMA_SCR_MBURST_INCR4)
#define SAI_DMA_RX_CR     (DMA_SCR_PFCTRL | DMA_SCR_DIR_P2M)
#define SAI_DMA_TX_CR     (DMA_SCR_DIR_M2P)

/* SAI register can only handle 32bit writes. If PSIZE is smaller then data
 * will be duplicated.
 */
#define SAI_DMA_8BIT_CR   (DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_8BITS)
#define SAI_DMA_16BIT_CR  (DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_16BITS)
#define SAI_DMA_32BIT_CR  (DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS)

/* If built with CONFIG_ARMV7M_DCACHE Buffers need to be aligned and
 * multiples of ARMV7M_DCACHE_LINESIZE
 */

#define SAI_BUFFER_ADJUSTED_SIZE DCACHE_BUFFER_SIZE_ADJUST(CONFIG_STM32H7_SAI_BDMA_BUFFER_SIZE)


#define SAI_NUMBER_OF_BUFFERS CONFIG_STM32H7_SAI_USER_BUFFERS
#define SAI_NUMBER_OF_CONTAINERS SAI_NUMBER_OF_BUFFERS

/* Only one audio driver can use the SAI interface.
 * In sync mode we allow the audio driver to reserve the SAI twice. Once for
 * each audio direction.
 */

#define SAI_MAX_RESERVE 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Audio buffer container */

struct sai_buffer_s
{
  struct sai_buffer_s *flink; /* Supports a singly linked list */
  i2s_callback_t callback;    /* Function to call when the transfer completes */
  void *arg;                  /* The argument to return with the callback */
  struct ap_buffer_s *apb;    /* The audio buffer */
  int result;                 /* The result of the transfer */
};

/* The state of the SAI block
 *
 * If two SAI interfaces, using internal synchronization, is used for
 * full-duplex communication. One interface is always RX and the other always
 * TX.
 */

struct stm32h7_sai_s
{
  struct i2s_dev_s dev;        /* Externally visible I2S interface */

  uint8_t sai_block;           /* SAI interface number */
  uint8_t master:1;            /* Master or Slave interface */
  uint8_t tx:1;                /* This interface is TX. Otherwise RX */
  uint8_t reserved:2;          /* Only one driver may use the interface */
  uint8_t started:1;           /* Audio streaming has been started */
  uint8_t ct:1;                /* Current Target in double buffer */
  struct sai_format_s *format; /* Audio format configuration */
  uintptr_t base;              /* SAI block register base address */
  mutex_t lock;                /* Assures mutually exclusive access to SAI */
  uint8_t irq;                 /* SAI IRQ number */
  uint16_t dma_ch;             /* DMA channel number */
  DMA_HANDLE dma;              /* DMA channel handle */
  sq_queue_t pend;             /* A queue of pending transfers */
  mutex_t queue_lock;          /* Assures mutually exclusive access to queues */
  int transfer_result;         /* The result code from the last transfer */
  struct work_s work;          /* Supports worker thread operations */
  struct stm32h7_sai_s *sync_priv;  /* Pointer to synchronized SAI interface */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                   /* Buffer wait semaphore */
  struct sai_buffer_s *freelist;  /* A list of free buffer containers */
  struct sai_buffer_s containers[SAI_NUMBER_OF_CONTAINERS];

  /* Pre-allocated pool of audio buffers */

  sq_queue_t apb_freelist;
  struct ap_buffer_s apb_buffers[SAI_NUMBER_OF_BUFFERS];
  uint8_t apb_samps[(SAI_BUFFER_ADJUSTED_SIZE * SAI_NUMBER_OF_BUFFERS)];
  uint8_t *dma_buffer;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
static void     sai_dump_regs(struct stm32h7_sai_s *priv, const char *msg);
#else
#  define       sai_dump_regs(s,m)
#endif

/* Enable/disable helpers */

static void sai_disable_block(struct stm32h7_sai_s *priv);
static void sai_enable_block(struct stm32h7_sai_s *priv);
static void sai_disable(struct stm32h7_sai_s *priv);
static void sai_enable(struct stm32h7_sai_s *priv);
static void sai_disable_dma(struct stm32h7_sai_s *priv);
static void sai_enable_dma(struct stm32h7_sai_s *priv);

/* Buffer container helpers */

static struct sai_buffer_s *sai_container_allocate(struct stm32h7_sai_s *priv);
static void sai_container_free(struct stm32h7_sai_s *priv,
                         struct sai_buffer_s *bfcontainer);
static void sai_buf_initialize(struct stm32h7_sai_s *priv);

/* DMA support */

static void sai_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg);

/* I2S methods */

static int sai_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout);
static int sai_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout);
static int sai_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  .i2s_receive      = sai_receive,
  .i2s_send         = sai_send,
  .i2s_ioctl        = sai_ioctl,
};

#if defined(CONFIG_STM32H7_SAI1_A)
#error TODO: place buffers in correct SRAM, dictated by DMA
static struct stm32h7_sai_s g_sai1a_priv;
uint8_t g_sai1a_dma_buffer[(SAI_BUFFER_ADJUSTED_SIZE * 2)] DCACHE_BUFFER_ALIGN locate_data(".sramX");
#endif
#if defined(CONFIG_STM32H7_SAI1_B)
#error TODO: place buffers in correct SRAM, dictated by DMA
static struct stm32h7_sai_s g_sai1b_priv;
uint8_t g_sai1b_dma_buffer[(SAI_BUFFER_ADJUSTED_SIZE * 2)] DCACHE_BUFFER_ALIGN locate_data(".sramX");
#endif
#if defined(CONFIG_STM32H7_SAI4_A)
static struct stm32h7_sai_s g_sai4a_priv;
uint8_t g_sai4a_dma_buffer[(SAI_BUFFER_ADJUSTED_SIZE * 2)] DCACHE_BUFFER_ALIGN locate_data(".sram4");
#endif
#if defined(CONFIG_STM32H7_SAI4_B)
static struct stm32h7_sai_s g_sai4b_priv;
uint8_t g_sai4b_dma_buffer[(SAI_BUFFER_ADJUSTED_SIZE * 2)] DCACHE_BUFFER_ALIGN locate_data(".sram4");
#endif

/* SAI1 state */

#ifdef CONFIG_STM32H7_SAI1_A
static struct stm32h7_sai_s g_sai1a_priv =
{
  .dev.ops        = &g_i2sops,
  .sai_block      = SAI1_BLOCK_A,
#if defined(CONFIG_STM32H7_SAI1_A_SYNC_WITH_B)
  .master         = 0,
#elif defined(CONFIG_STM32H7_SAI1_B_SYNC_WITH_A)
  .master         = 1,
#endif
  .sync_priv      = &g_sai1b_priv,
  .base           = STM32H7_SAI1_A_BASE,
  .lock           = NXMUTEX_INITIALIZER,
  .irq            = STM32_IRQ_SAI1,
  .dma_ch         = DMACHAN_SAI1_A,
  .queue_lock     = NXMUTEX_INITIALIZER,
  .reserved       = 0,
  .bufsem         = SEM_INITIALIZER(0),
  .dma_buffer     = g_sai1a_dma_buffer,
};
#endif

#ifdef CONFIG_STM32H7_SAI1_B
static struct stm32h7_sai_s g_sai1b_priv =
{
  .dev.ops        = &g_i2sops,
  .sai_block      = SAI1_BLOCK_B,
#if defined(CONFIG_STM32H7_SAI1_A_SYNC_WITH_B)
  .master         = 1,
#elif defined(CONFIG_STM32H7_SAI1_B_SYNC_WITH_A)
  .master         = 0,
#endif
  .sync_priv      = &g_sai1a_priv,
  .base           = STM32H7_SAI1_B_BASE,
  .lock           = NXMUTEX_INITIALIZER,
  .irq            = STM32_IRQ_SAI1,
  .dma_ch         = DMACHAN_SAI1_B,
  .queue_lock     = NXMUTEX_INITIALIZER,
  .reserved       = 0,
  .bufsem         = SEM_INITIALIZER(0),
  .dma_buffer     = g_sai1b_dma_buffer,
};
#endif

/* SAI4 state */

#ifdef CONFIG_STM32H7_SAI4_A
static struct stm32h7_sai_s g_sai4a_priv =
{
  .dev.ops        = &g_i2sops,
  .sai_block      = SAI4_BLOCK_A,
#if defined(CONFIG_STM32H7_SAI4_A_SYNC_WITH_B)
  .master         = 0,
#elif defined(CONFIG_STM32H7_SAI4_B_SYNC_WITH_A)
  .master         = 1,
#endif
  .sync_priv      = &g_sai4b_priv,
  .base           = STM32H7_SAI4_A_BASE,
  .lock           = NXMUTEX_INITIALIZER,
  .irq            = STM32_IRQ_SAI4,
  .dma_ch         = DMAMAP_BDMA_SAI4A,
  .queue_lock     = NXMUTEX_INITIALIZER,
  .reserved       = 0,
  .bufsem         = SEM_INITIALIZER(0),
  .dma_buffer     = g_sai4a_dma_buffer,
};
#endif

#ifdef CONFIG_STM32H7_SAI4_B
static struct stm32h7_sai_s g_sai4b_priv =
{
  .dev.ops        = &g_i2sops,
  .sai_block      = SAI4_BLOCK_B,
#if defined(CONFIG_STM32H7_SAI4_A_SYNC_WITH_B)
  .master         = 1,
#elif defined(CONFIG_STM32H7_SAI4_B_SYNC_WITH_A)
  .master         = 0,
#endif
  .sync_priv      = &g_sai4a_priv,
  .base           = STM32H7_SAI4_B_BASE,
  .lock           = NXMUTEX_INITIALIZER,
  .irq            = STM32_IRQ_SAI4,
  .dma_ch         = DMAMAP_BDMA_SAI4B,
  .queue_lock     = NXMUTEX_INITIALIZER,
  .reserved       = 0,
  .bufsem         = SEM_INITIALIZER(0),
  .dma_buffer     = g_sai4b_dma_buffer,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sai_getreg
 *
 * Description:
 *   Get the contents of the SAI register at offset
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t sai_getreg(struct stm32h7_sai_s *priv, uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sai_putreg
 *
 * Description:
 *   Write a 32-bit value to the SAI register at offset
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sai_putreg(struct stm32h7_sai_s *priv, uint8_t offset,
                              uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: sai_modifyreg
 *
 * Description:
 *   Clear and set bits in the SAI register at offset
 *
 * Input Parameters:
 *   priv    - private SAI device structure
 *   offset  - offset to the register of interest
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_modifyreg(struct stm32h7_sai_s *priv, uint8_t offset,
                          uint32_t clrbits, uint32_t setbits)
{
  uint32_t regval;

  regval  = sai_getreg(priv, offset);
  regval &= ~clrbits;
  regval |= setbits;
  sai_putreg(priv, offset, regval);
}

/****************************************************************************
 * Name: sai_dump_regs
 *
 * Description:
 *   Dump the contents of all SAI block registers
 *
 * Input Parameters:
 *   priv - The SAI block controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
static void sai_dump_regs(struct stm32h7_sai_s *priv, const char *msg)
{
  if (msg)
      i2sinfo("%s\n", msg);

#if defined(CONFIG_STM32H7_SAI1_A) || defined(CONFIG_STM32H7_SAI1_B)
  uint32_t gcr = getreg32(STM32H7_SAI1_GCR);
  i2sinfo("GCR: *%08x = %08x\n", STM32H7_SAI1_GCR, gcr);
#endif
#if defined(CONFIG_STM32H7_SAI4_A) || defined(CONFIG_STM32H7_SAI4_B)
  uint32_t gcr = getreg32(STM32H7_SAI4_GCR);
  i2sinfo("GCR: *%08x = %08x\n", STM32H7_SAI4_GCR, gcr);
#endif
  i2sinfo("CR1:%08x CR2:%08x  FRCR:%08x SLOTR:%08x\n",
          sai_getreg(priv, STM32H7_SAI_CR1_OFFSET),
          sai_getreg(priv, STM32H7_SAI_CR2_OFFSET),
          sai_getreg(priv, STM32H7_SAI_FRCR_OFFSET),
          sai_getreg(priv, STM32H7_SAI_SLOTR_OFFSET));
  i2sinfo(" IM:%08x  SR:%08x CLRFR:%08x\n",
          sai_getreg(priv, STM32H7_SAI_IM_OFFSET),
          sai_getreg(priv, STM32H7_SAI_SR_OFFSET),
          sai_getreg(priv, STM32H7_SAI_CLRFR_OFFSET));
}
#endif

/****************************************************************************
 * Name: sai_pwr_disable
 *
 * Description:
 *   Turn off clock for SAI interface and (B)DMA.
 *   Will power off both blocks belonging to an interface, since they share
 *   clock and DMA.
 *
 * Input Parameters:
 *   intf - The SAI block that should be powered down
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_pwr_disable(int intf)
{
  uint32_t regval;

  i2sinfo("Entry");

#if defined(CONFIG_STM32H7_SAI4_A) || defined(CONFIG_STM32H7_SAI4_B)
  if (intf == SAI4_BLOCK_A || intf == SAI4_BLOCK_B)
    {
      regval = getreg32(STM32_RCC_APB4ENR);
      regval &= ~RCC_APB4ENR_SAI4EN;
      putreg32(regval, STM32_RCC_APB4ENR);

      regval = getreg32(STM32_RCC_AHB4ENR);
      regval &= ~RCC_AHB4ENR_BDMAEN;
      putreg32(regval, STM32_RCC_AHB4ENR);
    }
#endif

#if defined(CONFIG_STM32H7_SAI1_A) || defined(CONFIG_STM32H7_SAI1_B)
#    error "TODO: Clock and DMA disable for SAI1 missing"
#endif
}

/****************************************************************************
 * Name: sai_pwr_enable
 *
 * Description:
 *   Turn on clock for SAI interface and (B)DMA.
 *   Will power on both blocks belonging to an interface, since they share
 *   clock and DMA.
 *
 * Input Parameters:
 *   intf - The SAI block that should be powered up
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_pwr_enable(int intf)
{
  uint32_t regval;

  i2sinfo("Entry");

#if defined(CONFIG_STM32H7_SAI4_A) || defined(CONFIG_STM32H7_SAI4_B)
  if (intf == SAI4_BLOCK_A || intf == SAI4_BLOCK_B)
    {
      regval = getreg32(STM32_RCC_APB4ENR);
      regval |= RCC_APB4ENR_SAI4EN;
      putreg32(regval, STM32_RCC_APB4ENR);

      regval = getreg32(STM32_RCC_AHB4ENR);
      regval |= RCC_AHB4ENR_BDMAEN;
      putreg32(regval, STM32_RCC_AHB4ENR);
    }
#endif

#if defined(CONFIG_STM32H7_SAI1_A) || defined(CONFIG_STM32H7_SAI1_B)
#   error "TODO: Clock and DMA enable for SAI1 missing"
#endif
}

/****************************************************************************
 * Name: sai_disable_block
 *
 * Description:
 *   Put single SAI block in disabled mode.
 *   After enable bit has been cleared SAI will continue to finish any RX/TX
 *   operations. When done, SAI will clear the enable bit in the register.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_disable_block(struct stm32h7_sai_s *priv)
{
  uint32_t regcr1;

  DEBUGASSERT(priv);

  regcr1 = sai_getreg(priv, STM32H7_SAI_CR1_OFFSET);

  if ((regcr1 & SAI_CR1_SAIEN) != 0)
    {
      regcr1 &= ~SAI_CR1_SAIEN;
      sai_putreg(priv, STM32H7_SAI_CR1_OFFSET, regcr1);

      do
        {
          /* SAI will be disabled when finished with current RX/TX. */

          usleep(10);
          regcr1 = sai_getreg(priv, STM32H7_SAI_CR1_OFFSET);
        } while (regcr1 & SAI_CR1_SAIEN);
    }
}

/****************************************************************************
 * Name: sai_enable_block
 *
 * Description:
 *   Enable single SAI block.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_enable_block(struct stm32h7_sai_s *priv)
{
  uint32_t regcr1;

  DEBUGASSERT(priv);

  regcr1 = sai_getreg(priv, STM32H7_SAI_CR1_OFFSET);

  if ((regcr1 & SAI_CR1_SAIEN) == 0)
    {
      regcr1 |= SAI_CR1_SAIEN;
      sai_putreg(priv, STM32H7_SAI_CR1_OFFSET, regcr1);
    }
}

/****************************************************************************
 * Name: sai_disable
 *
 * Description:
 *   Disable both blocks belonging to the same SAI interface. In synchronous
 *   mode Master and Slave must be en-/dis-abled in specific order.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_disable(struct stm32h7_sai_s *priv)
{
  struct stm32h7_sai_s *priv_master;
  struct stm32h7_sai_s *priv_slave;

  i2sinfo("Entry");

  DEBUGASSERT(priv && priv->sync_priv);

  priv_master = (priv->master) ? priv : priv->sync_priv;
  priv_slave = (priv->master) ? priv->sync_priv : priv;

  sai_disable_block(priv_master);
  sai_disable_block(priv_slave);
}

/****************************************************************************
 * Name: sai_enable
 *
 * Description:
 *   Enable both blocks belonging to the same SAI interface. In synchronous
 *   mode Master and Slave must be en-/dis-abled in specific order.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_enable(struct stm32h7_sai_s *priv)
{
  struct stm32h7_sai_s *priv_master;
  struct stm32h7_sai_s *priv_slave;

  i2sinfo("Entry");

  DEBUGASSERT(priv && priv->sync_priv);

  priv_master = (priv->master) ? priv : priv->sync_priv;
  priv_slave = (priv->master) ? priv->sync_priv : priv;

  sai_enable_block(priv_slave);
  sai_enable_block(priv_master);
}

/****************************************************************************
 * Name: sai_disable_dma
 *
 * Description:
 *   Disable DMA for both blocks belonging to the same SAI interface.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_disable_dma(struct stm32h7_sai_s *priv)
{
  DEBUGASSERT(priv && priv->sync_priv);

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_DMAEN, 0);
  sai_modifyreg(priv->sync_priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_DMAEN, 0);

  sai_modifyreg(priv, STM32H7_SAI_CR2_OFFSET, 0, SAI_CR2_FFLUSH);
  sai_modifyreg(priv->sync_priv, STM32H7_SAI_CR2_OFFSET, 0, SAI_CR2_FFLUSH);

  sai_modifyreg(priv, STM32H7_SAI_CLRFR_OFFSET, 0, 0xFFFFFFFF);
  sai_modifyreg(priv->sync_priv, STM32H7_SAI_CLRFR_OFFSET, 0, 0xFFFFFFFF);
}

/****************************************************************************
 * Name: sai_enable_dma
 *
 * Description:
 *   Enable DMA for both blocks belonging to the same SAI interface.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_enable_dma(struct stm32h7_sai_s *priv)
{
  struct stm32h7_sai_s *priv_tx;
  uint32_t fifolvl;

  DEBUGASSERT(priv && priv->sync_priv);

  priv_tx = (priv->tx) ? priv : priv->sync_priv;

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, 0, SAI_CR1_DMAEN);
  sai_modifyreg(priv->sync_priv, STM32H7_SAI_CR1_OFFSET, 0, SAI_CR1_DMAEN);

  do
    {
      /* Wait for TX FIFO to have content. */

      fifolvl = sai_getreg(priv_tx, STM32H7_SAI_SR_OFFSET) & SAI_INT_FLVL_MASK;
    } while (fifolvl == SAI_INT_FLVL_EMPTY);
}

/****************************************************************************
 * Name: sai_clear_queue
 *
 * Description:
 *   Clear single queue of SAI buffer containers.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *   q - buffer container queue to be cleared
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_clear_queue(struct stm32h7_sai_s *priv, sq_queue_t *q)
{
  struct sai_buffer_s *bfcontainer;

  while (!sq_empty(q))
    {
      bfcontainer = (struct sai_buffer_s *)sq_remfirst(q);
      sai_container_free(priv, bfcontainer);
    }
}

/****************************************************************************
 * Name: sai_clear_all_queues
 *
 * Description:
 *   Clear all queues of SAI buffer containers.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_clear_all_queues(struct stm32h7_sai_s *priv)
{
  nxmutex_lock(&priv->queue_lock);
  sai_clear_queue(priv, &priv->pend);
  nxmutex_unlock(&priv->queue_lock);
  nxmutex_lock(&priv->sync_priv->queue_lock);
  sai_clear_queue(priv->sync_priv, &priv->sync_priv->pend);
  nxmutex_unlock(&priv->sync_priv->queue_lock);
}

/****************************************************************************
 * Name: sai_fill_buffer_rx
 *
 * Description:
 *   Copy data from DMA input buffer to user space audio buffer and return the
 *   audio buffer to user space.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sai_fill_buffer_rx(struct stm32h7_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;

  nxmutex_lock(&priv->queue_lock);
  bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->pend);
  nxmutex_unlock(&priv->queue_lock);

  /* If we don't have a buffer from userspace then we just forget about the
   * data we just received.
   * We don't test for buffer sizes, so make sure that userspace allocated
   * buffers through this driver.
   */

  if (bfcontainer)
    {
      memcpy(bfcontainer->apb->samp,
             &priv->dma_buffer[(priv->ct * SAI_BUFFER_ADJUSTED_SIZE)],
             SAI_BUFFER_ADJUSTED_SIZE);

      /* Perform the transfer done callback */

      DEBUGASSERT(bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, priv->transfer_result);

      sai_container_free(priv, bfcontainer);
    }
  else
    {
      i2swarn("No RX buffer available");
    }

  /* Change current target of double buffer */

  priv->ct ^= 1;
}

/****************************************************************************
 * Name: sai_fill_buffer_tx
 *
 * Description:
 *   Copy data from the user space audio buffer to the DMA buffer and return
 *   the audio buffer to user space immediately, for reuse.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sai_fill_buffer_tx(struct stm32h7_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;

  nxmutex_lock(&priv->queue_lock);
  bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->pend);
  nxmutex_unlock(&priv->queue_lock);

  /* If we don't have a buffer from userspace then we just send 0 values.
   * We don't test for buffer sizes, so make sure that userspace allocated
   * buffers through this driver.
   */

  if (bfcontainer)
    {
      memcpy(&priv->dma_buffer[(priv->ct * SAI_BUFFER_ADJUSTED_SIZE)],
             bfcontainer->apb->samp, SAI_BUFFER_ADJUSTED_SIZE);

      /* Return buffer to userspace immediately. Transfer result is actually
       * the result of the previous transfer, since we're not waiting for the
       * transfer to complete before returning the buffer.
       */

      DEBUGASSERT(bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, priv->transfer_result);

      sai_container_free(priv, bfcontainer);
    }
  else
    {
      i2swarn("No TX buffer available. Sending value 0.");
      memset(&priv->dma_buffer[(priv->ct * SAI_BUFFER_ADJUSTED_SIZE)], 0,
             SAI_BUFFER_ADJUSTED_SIZE);
    }

  /* Change current target of double buffer */

  priv->ct ^= 1;
}

/****************************************************************************
 * Name: sai_fill_buffer
 *
 * Description:
 *   Forward buffer call to appropriate function depending on transfer
 *   direction.
 *
 * Input Parameters:
 *   priv - SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sai_fill_buffer(struct stm32h7_sai_s *priv)
{
  if (priv->tx)
    {
      sai_fill_buffer_tx(priv);
    }
  else
    {
      sai_fill_buffer_rx(priv);
    }
}


/****************************************************************************
 * Name: sai_dma_channel_configure
 *
 * Description:
 *   Configure the (B)DMA for a single SAI block's transfer size and direction.
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_dma_channel_configure(struct stm32h7_sai_s *priv)
{
  stm32_dmacfg_t dmacfg;
  apb_samp_t nbytes = SAI_BUFFER_ADJUSTED_SIZE;
  uint32_t ntransfers = 0;
  uint32_t dma_ccr = SAI_DMA_BASE_CR;

  if (priv->tx)
    {
      dma_ccr |= SAI_DMA_TX_CR;
    }
  else
    {
      dma_ccr |= SAI_DMA_RX_CR;
    }

  switch (priv->format->datasize)
    {
      case 8:
        dma_ccr |= SAI_DMA_8BIT_CR;
        ntransfers = nbytes;
        break;

      default:
      case 16:
        dma_ccr |= SAI_DMA_16BIT_CR;
        DEBUGASSERT((nbytes & 0x1) == 0);
        ntransfers = nbytes >> 1;
        break;

      case 32:
        dma_ccr |= SAI_DMA_32BIT_CR;
        DEBUGASSERT((nbytes & 0x3) == 0);
        ntransfers = nbytes >> 2;
        break;
    }

  DEBUGASSERT(ntransfers > 0);

  /* Always start with the first of the double buffers as current target */

  priv->ct = 0;

  dmacfg.paddr = priv->base + STM32H7_SAI_DR_OFFSET;
  dmacfg.maddr = (uintptr_t)priv->dma_buffer;
  dmacfg.ndata = ntransfers;
  dmacfg.cfg1  = dma_ccr;
  dmacfg.cfg2  = 0;

  stm32_dmasetup(priv->dma, &dmacfg);

  /* Enable DMA. Transfer won't start until SAI has been enabled as well. */

  stm32_dmastart(priv->dma, sai_dma_callback, priv, false);
}

/****************************************************************************
 * Name: sai_stop
 *
 * Description:
 *   Disable both SAI blocks. If transfers are ongoing this function will block
 *   until they are finished.
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sai_stop(struct stm32h7_sai_s *priv)
{
  priv->started = 0;

  sai_disable(priv);
  sai_disable_dma(priv);

  if (priv->reserved == 0 && priv->sync_priv->reserved == 0)
    {
      /* Reserved means that a higher layer is using the driver. If no one is
       * using it we should power off.
       */

      i2swarn("driver not reserved");
      sai_clear_all_queues(priv);
      sai_pwr_disable(priv->sai_block);
    }

  return OK;
}

/****************************************************************************
 * Name: sai_start
 *
 * Description:
 *   Enable both SAI blocks, thus starting transfers.
 *   Note that it is possible for user space to queue audio buffers before
 *   calling this function.
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline int sai_start(struct stm32h7_sai_s *priv)
{
  struct stm32h7_sai_s *priv_tx;

  if (priv->reserved == 0 && priv->sync_priv->reserved == 0)
    {
      /* Reserved means that a higher layer is using the driver. If no one is
       * using it we should power off.
       */

      i2swarn("driver not reserved");
      sai_clear_all_queues(priv);
      sai_pwr_disable(priv->sai_block);
      return -EACCES;
    }

  if (priv->started)
    {
      i2sinfo("transmission already active");
      return OK;
    }

  priv->started = 1;

  sai_disable(priv);
  sai_disable_dma(priv);

  sai_dma_channel_configure(priv);
  sai_dma_channel_configure(priv->sync_priv);

  /* User may have prepared audio output by placing buffers in queue. Otherwise
   * we need to zero the DMA buffer anyways, so call the function to fill the
   * TX buffer twice in order to fill both DMA buffers in our double buffer
   * setup.
   */

  priv_tx = (priv->tx) ? priv : priv->sync_priv;
  sai_fill_buffer(priv_tx);
  sai_fill_buffer(priv_tx);

  sai_enable_dma(priv);
  sai_enable(priv);

  i2sinfo("Done");

  return OK;
}

/****************************************************************************
 * Name: sai_worker
 *
 * Description:
 *   Transfer worker, scheduled from DMA callback (interrupt context).
 *
 * Input Parameters:
 *   arg - the SAI device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_worker(void *arg)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)arg;

  DEBUGASSERT(priv);

  sai_fill_buffer(priv);
}

/****************************************************************************
 * Name: sai_dma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the SAI DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   isr    - The interrupt status of the DMA transfer
 *   arg    - A pointer to the SAI state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)arg;
  int ret = ERROR;

  DEBUGASSERT(priv);

  priv->transfer_result = (isr & DMA_STATUS_TEIF) ? -EIO : OK;

  /* Schedule the done processing to occur on the worker thread. */

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, sai_worker, priv, 0);
    }
  if (ret != OK)
    {
      i2serr("ERROR: Failed to queue work: %d\n", ret);
    }
}

/****************************************************************************
 * Name: sai_queue_transfer
 *
 * Description:
 *   Queue an audio buffer for transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the user space audio buffer
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use. We ignore this since we are running
 *              continously in circular double buffer mode.
 *   tx       - Transfer direction. True: TX, false: RX
 *
 * Returned Value:
 *   OK.
 *
 *   NOTE: This function only enqueues the transfer and returns
 *   immediately. Success here only means that the transfer was enqueued
 *   correctly.
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int sai_queue_transfer(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                              i2s_callback_t callback, void *arg,
                              uint32_t timeout, uint8_t tx)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)dev;
  struct sai_buffer_s *bfcontainer;

  UNUSED(timeout);

  DEBUGASSERT(priv && apb);

  /* In sync mode only the Master should receive commands. */

  DEBUGASSERT(priv->master);

  if (priv->tx != tx)
    {
      /* The Slave might be responsible for the transfer direction. */

      priv = (struct stm32h7_sai_s *)priv->sync_priv;
      DEBUGASSERT(priv);
    }

  bfcontainer = sai_container_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Add the buffer to a container and add the container to the end of the
   * pending queue
   */

  nxmutex_lock(&priv->lock);

  bfcontainer->callback = (void *)callback;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  nxmutex_lock(&priv->queue_lock);
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);
  nxmutex_unlock(&priv->queue_lock);

  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: sai_receive
 *
 * Description:
 *   Queue a buffer for receiving audio data.
 *
 * Input Parameters:
 *   See sai_queue_transfer()
 *
 * Returned Value:
 *   See sai_queue_transfer()
 *
 ****************************************************************************/

static int sai_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  i2sinfo("Entry");
  return sai_queue_transfer(dev, apb, callback, arg, timeout, 0);
}

/****************************************************************************
 * Name: sai_send
 *
 * Description:
 *   Queue a buffer for sending audio data.
 *
 * Input Parameters:
 *   See sai_queue_transfer()
 *
 * Returned Value:
 *   See sai_queue_transfer()
 *
 ****************************************************************************/

static int sai_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  i2sinfo("Entry");
  return sai_queue_transfer(dev, apb, callback, arg, timeout, 1);
}

/****************************************************************************
 * Name: sai_ioctl
 *
 * Description:
 *   IOCTL commands.
 *   These commands are not intended for user space control but for
 *   interaction with an audio driver.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   cmd      - Command values AUDIOIOC_* from audio.h are used
 *   arg      - Pointer to data needed by command, cast to unsigned long
 *
 * Returned Value:
 *   OK on success or negative error value
 *
 ****************************************************************************/

static int sai_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)dev;
  struct audio_buf_desc_s *bufdesc;
  struct ap_buffer_s *apb;
  int ret = -ENOTTY;

  switch (cmd)
    {
    case AUDIOIOC_RESERVE:
      {
        i2sinfo("AUDIOIOC_RESERVE");

        ret = OK;
        nxmutex_lock(&priv->lock);
        if (priv->reserved >= SAI_MAX_RESERVE)
          {
            i2serr("Already reserved");
            ret = -EBUSY;
          }
        else
          {
            if (!priv->reserved)
              {
                sai_pwr_enable(priv->sai_block);
              }
            priv->reserved += 1;
          }
        nxmutex_unlock(&priv->lock);
      }
      break;
    case AUDIOIOC_RELEASE:
      {
        i2sinfo("AUDIOIOC_RELEASE");

        ret = OK;
        nxmutex_lock(&priv->lock);
        priv->reserved -= 1;
        DEBUGASSERT(priv->reserved >= 0);
        if (priv->reserved == 0 && priv->started)
          {
            ret = sai_stop(priv);
          }
        nxmutex_unlock(&priv->lock);
      }
      break;
    case AUDIOIOC_START:
      {
        i2sinfo("AUDIOIOC_START");

        nxmutex_lock(&priv->lock);
        ret = sai_start(priv);
        nxmutex_unlock(&priv->lock);
      }
      break;
    case AUDIOIOC_STOP:
      {
        i2sinfo("AUDIOIOC_STOP");

        nxmutex_lock(&priv->lock);
        ret = sai_stop(priv);
        nxmutex_unlock(&priv->lock);
      }
      break;
    case AUDIOIOC_ALLOCBUFFER:
      {
        i2sinfo("AUDIOIOC_ALLOCBUFFER");

        nxmutex_lock(&priv->lock);
        apb = (struct ap_buffer_s *)sq_remfirst(&priv->apb_freelist);
        if (apb != NULL)
          {
            bufdesc = (struct audio_buf_desc_s *)arg;
            bufdesc->u.buffer = apb;
            i2sinfo("samp addr: 0x%x", (unsigned int)bufdesc->u.buffer->samp);

            /* Ignore request values for buffer size. We created the buffers
             * at compilation time.
             */

            apb->nmaxbytes  = SAI_BUFFER_ADJUSTED_SIZE;
            apb->crefs      = 1;
            apb->nbytes     = 0;
            apb->flags      = 0;
      #ifdef CONFIG_AUDIO_MULTI_SESSION
            apb->session    = bufdesc->session;
      #endif
            ret = sizeof(struct audio_buf_desc_s);
          }
        else
          {
            i2serr("No buffer mem available");
            ret = -ENOMEM;
          }
        nxmutex_unlock(&priv->lock);
      }
      break;
    case AUDIOIOC_FREEBUFFER:
      {
        i2sinfo("AUDIOIOC_FREEBUFFER");

        nxmutex_lock(&priv->lock);
        bufdesc = (struct audio_buf_desc_s *)arg;
        DEBUGASSERT(bufdesc->u.buffer != NULL);
        sq_addfirst((sq_entry_t *)bufdesc->u.buffer, &priv->apb_freelist);
        ret = sizeof(struct audio_buf_desc_s);
        nxmutex_unlock(&priv->lock);
      }
      break;

    default:
      break;
  }

return ret;
}

/****************************************************************************
 * Name: sai_interrupt
 *
 * Description:
 *   Prints status register on error interrupt
 *
 * Input Parameters:
 *   irq     -
 *   context -
 *   arg     - the SAI device instance cast to void*
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int sai_interrupt(int irq, void *context, void *arg)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)arg;
  uint32_t sr = sai_getreg(priv, STM32H7_SAI_SR_OFFSET);

  i2serr("Status: 0x%08x\n", sr);

  return OK;
}

/****************************************************************************
 * Name: sai_container_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the SAI state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct sai_buffer_s *sai_container_allocate(struct stm32h7_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  nxsem_wait_uninterruptible(&priv->bufsem);

  /* Get the buffer from the head of the free list */

  flags = enter_critical_section();
  bfcontainer = priv->freelist;
  DEBUGASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->freelist = bfcontainer->flink;
  leave_critical_section(flags);

  return bfcontainer;
}

/****************************************************************************
 * Name: sai_container_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the SAI state structure
 *
 ****************************************************************************/

static void sai_container_free(struct stm32h7_sai_s *priv,
                               struct sai_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = enter_critical_section();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  leave_critical_section(flags);

  /* Wake up any threads waiting for a buffer container */

  nxsem_post(&priv->bufsem);
}

/****************************************************************************
 * Name: sai_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in SAI initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void sai_buf_initialize(struct stm32h7_sai_s *priv)
{
  int i;
  struct stm32h7_sai_s *priv_master = (priv->master) ? priv : priv->sync_priv;

  /* Place buffer containers used locally in freelist */

  priv->freelist = NULL;
  for (i = 0; i < SAI_NUMBER_OF_CONTAINERS; i++)
    {
      sai_container_free(priv, &priv->containers[i]);
    }


  /* Assign sample buffers to apb buffers. Place apb buffers in master's
   * freelist. apb buffers are passed to userspace through IOCTL allocation
   * calls from audio driver.
   */

  for (i = 0; i < SAI_NUMBER_OF_BUFFERS; i++)
    {
      priv->apb_buffers[i].samp =
                              &priv->apb_samps[(i*SAI_BUFFER_ADJUSTED_SIZE)];
      nxmutex_init(&priv->apb_buffers[i].lock);
      sq_addfirst((sq_entry_t *)&priv->apb_buffers[i],
                  &priv_master->apb_freelist);
    }
}

/****************************************************************************
 * Name: sai_configure
 *
 * Description:
 *   Configure the selected SAI block
 *
 * Input Parameters:
 *   priv   - SAI state instance
 *   format - Pointer to static struct that describes the audio format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_configure(struct stm32h7_sai_s *priv,
                          struct sai_format_s *format)
{
  uint32_t regcr1;
  uint32_t regfrcr;
  uint32_t regslot;
  uint32_t regim;
  uint32_t framelen;
  uint32_t frameactive;

  DEBUGASSERT(priv && format);

  sai_disable(priv);

  regcr1 = sai_getreg(priv, STM32H7_SAI_CR1_OFFSET);

  DEBUGASSERT(format->nslots <= 16);

  regfrcr = sai_getreg(priv, STM32H7_SAI_FRCR_OFFSET);
  regslot = sai_getreg(priv, STM32H7_SAI_SLOTR_OFFSET);

  if (format->nslots != 0 && format->nslots <= 16)
    {
      /* TDM */

      DEBUGASSERT(format->leftslot <= format->nslots);
      DEBUGASSERT(format->leftslot > 0);
      DEBUGASSERT(format->rightslot <= format->nslots);

      /* Frame register */

      framelen = format->nslots * format->slotsize;
      if (format->nodiv == 0)
        {
          /* If nodiv is 0, then frame length must be a power of 2. */

          uint32_t fl = 8;
          while (framelen > fl)
            {
              fl = fl << 1;
            }
          framelen = fl;
        }
      DEBUGASSERT(framelen >= 8 && framelen <= 256);

      frameactive = 1;

      regfrcr &= ~SAI_FRCR_FSDEF;
      regfrcr |= SAI_FRCR_FSPOL;
      regfrcr |= SAI_FRCR_FSOFF;

      /* Slot register */

      regslot &= ~SAI_SLOTR_SLOTSZ_MASK;
      switch (format->slotsize)
        {
          case 16:
            regslot |= SAI_SLOTR_SLOTSZ_16BIT;
            break;

          case 32:
            regslot |= SAI_SLOTR_SLOTSZ_32BIT;
            break;

          default:
            regslot |= SAI_SLOTR_SLOTSZ_DATA;
            break;
        }

      regslot &= ~SAI_SLOTR_NBSLOT_MASK;
      regslot |= SAI_SLOTR_NBSLOT(format->nslots);

      regslot &= ~SAI_SLOTR_SLOTEN_MASK;
      regslot |= SAI_SLOTR_SLOTEN(format->leftslot);
      if (format->rightslot > 0)
        {
          regslot |= SAI_SLOTR_SLOTEN(format->rightslot);
        }
    }
  else
    {
      /* I2S */

      /* Frame register */

      framelen = format->datasize * 2;
      frameactive = format->datasize;

      regfrcr |= SAI_FRCR_FSDEF_CHID;
      regfrcr &= ~SAI_FRCR_FSPOL;
      regfrcr |= SAI_FRCR_FSOFF_BFB;

      /* Slot register */

      regslot &= ~SAI_SLOTR_SLOTSZ_MASK;
      regslot |= SAI_SLOTR_SLOTSZ_DATA;

      regslot &= ~SAI_SLOTR_NBSLOT_MASK;
      regslot |= SAI_SLOTR_NBSLOT(2);

      regslot &= ~SAI_SLOTR_SLOTEN_MASK;
      regslot |= SAI_SLOTR_SLOTEN_0;
      regslot |= SAI_SLOTR_SLOTEN_1;
    }

  regfrcr &= ~SAI_FRCR_FRL_MASK;
  regfrcr |= SAI_FRCR_FRL(framelen);

  regfrcr &= ~SAI_FRCR_FSALL_MASK;
  regfrcr |= SAI_FRCR_FSALL(frameactive);

  sai_putreg(priv, STM32H7_SAI_FRCR_OFFSET, regfrcr);
  sai_putreg(priv, STM32H7_SAI_SLOTR_OFFSET, regslot);

  /* Configuration 1 register */

  i2sinfo("master: %d, mode: %s", priv->master, (priv->tx) ? "TX" : "RX");

  regcr1 &= ~SAI_CR1_MODE_MASK;
  if (priv->master)
    {
      if (priv->tx)
        {
          regcr1 |= SAI_CR1_MODE_MASTER_TX;
        }
      else
        {
          regcr1 |= SAI_CR1_MODE_MASTER_RX;
        }
    }
  else
    {
      if (priv->tx)
        {
          regcr1 |= SAI_CR1_MODE_SLAVE_TX;
        }
      else
        {
          regcr1 |= SAI_CR1_MODE_SLAVE_RX;
        }
    }

  regcr1 &= ~SAI_CR1_PRTCFG_MASK;             /* Free protocol */

  regcr1 &= ~SAI_CR1_DS_MASK;
  switch (format->datasize)
    {
      case 8:
        regcr1 |= SAI_CR1_DS_8BITS;
        break;

      default:
      case 16:
        regcr1 |= SAI_CR1_DS_16BITS;
        break;

      case 32:
        regcr1 |= SAI_CR1_DS_32BITS;
        break;
    }

  regcr1 &= ~SAI_CR1_LSBFIRST;
  regcr1 &= ~SAI_CR1_CKSTR;

  regcr1 &= ~SAI_CR1_SYNCEN_MASK;
  if (!priv->master)
    {
      regcr1 |= SAI_CR1_SYNCEN_INTERNAL;      /* Synchronous Slave */
    }

  regcr1 &= ~SAI_CR1_MONO;
  regcr1 |= SAI_CR1_OUTDRIV;

  regcr1 &= ~SAI_CR1_DMAEN;

  /* Clock */

  if (format->nodiv)
    {
      regcr1 |= SAI_CR1_NODIV;
    }
  else
    {
      regcr1 &= ~SAI_CR1_NODIV;
    }

  regcr1 &= ~SAI_CR1_MCKDIV_MASK;
  regcr1 |= SAI_CR1_MCKDIV(format->mckdiv);

  if (format->osr)
    {
      regcr1 |= SAI_CR1_OSR;
    }
  else
    {
      regcr1 &= ~SAI_CR1_OSR;
    }

  if (format->mcken)
    {
      regcr1 |= SAI_CR1_MCKEN;
    }
  else
    {
      regcr1 &= ~SAI_CR1_MCKEN;
    }

  sai_putreg(priv, STM32H7_SAI_CR1_OFFSET, regcr1);

  /* Configuration 2 register */

  if (priv->tx)
    {
      sai_modifyreg(priv, STM32H7_SAI_CR2_OFFSET, SAI_CR2_FTH_MASK,
                    SAI_CR2_FTH_1QF);
    }
  else
    {
      sai_modifyreg(priv, STM32H7_SAI_CR2_OFFSET, SAI_CR2_FTH_MASK,
                    SAI_CR2_FTH_1QF);
    }

  /* Interrupt mask register */

  regim = SAI_INT_OVRUDR;
  if (priv->master)
    {
      regim |= SAI_INT_WCKCFG;
    }
  else
    {
      regim |= SAI_INT_AFSDET | SAI_INT_LFSDET;
    }
  sai_putreg(priv, STM32H7_SAI_IM_OFFSET, regim);
}

/****************************************************************************
 * Name: sai_init
 *
 * Description:
 *   Initialize the selected SAI block
 *
 * Input Parameters:
 *   intf   - SAI block id
 *   format - Pointer to static struct that describes the audio format
 *   tx     - Transfer direction
 *
 * Returned Value:
 *   SAI instance private state
 *
 ****************************************************************************/

static struct stm32h7_sai_s *sai_init(int intf, struct sai_format_s *format,
                                      uint8_t tx)
{
  struct stm32h7_sai_s *priv = NULL;

  switch (intf)
    {
#ifdef CONFIG_STM32H7_SAI1_A
      case SAI1_BLOCK_A:
        {
          i2sinfo("SAI1 Block A Selected\n");
          priv = &g_sai1a_priv;

          stm32_configgpio(GPIO_SAI1_SD_A);
#  ifndef CONFIG_STM32H7_SAI1_A_SYNC_WITH_B
          stm32_configgpio(GPIO_SAI1_FS_A);
          stm32_configgpio(GPIO_SAI1_SCK_A);
          stm32_configgpio(GPIO_SAI1_MCLK_A);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32H7_SAI1_B
      case SAI1_BLOCK_B:
        {
          i2sinfo("SAI1 Block B Selected\n");
          priv = &g_sai1b_priv;

          stm32_configgpio(GPIO_SAI1_SD_B);
#  ifndef CONFIG_STM32H7_SAI1_B_SYNC_WITH_A
          stm32_configgpio(GPIO_SAI1_FS_B);
          stm32_configgpio(GPIO_SAI1_SCK_B);
          stm32_configgpio(GPIO_SAI1_MCLK_B);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32H7_SAI4_A
      case SAI4_BLOCK_A:
        {
          i2sinfo("SAI4 Block A Selected\n");
          priv = &g_sai4a_priv;

          stm32_configgpio(GPIO_SAI4_SD_A);
#  ifndef CONFIG_STM32H7_SAI4_A_SYNC_WITH_B
          stm32_configgpio(GPIO_SAI4_FS_A);
          stm32_configgpio(GPIO_SAI4_SCK_A);
          stm32_configgpio(GPIO_SAI4_MCLK_A);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32H7_SAI4_B
      case SAI4_BLOCK_B:
        {
          i2sinfo("SAI4 Block B Selected\n");
          priv = &g_sai4b_priv;

          stm32_configgpio(GPIO_SAI4_SD_B);
#  ifndef CONFIG_STM32H7_SAI4_B_SYNC_WITH_A
          stm32_configgpio(GPIO_SAI4_FS_B);
          stm32_configgpio(GPIO_SAI4_SCK_B);
          stm32_configgpio(GPIO_SAI4_MCLK_B);
#  endif
          break;
        }
#endif

      default:
        {
          i2serr("Could not find SAI interface %d\n", intf);
          goto err;
        }
    }

  priv->format = format;
  priv->tx = tx;

  sai_buf_initialize(priv);

  priv->dma = stm32_dmachannel(priv->dma_ch);
  DEBUGASSERT(priv->dma);

  sai_configure(priv, format);

  sai_dump_regs(priv, "After initialization");

err:
  return priv;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
                                              struct sai_format_s *format)
{
  struct stm32h7_sai_s *priv_rx;
  struct stm32h7_sai_s *priv_tx;
  irqstate_t flags;

  flags = enter_critical_section();
  sai_pwr_enable(block_tx);

  priv_rx = sai_init(block_rx, format, 0);
  priv_tx = sai_init(block_tx, format, 1);
  DEBUGASSERT(priv_rx && priv_tx && (priv_rx->master != priv_tx->master));

  sai_pwr_disable(block_tx);

  /* Attach the IRQ to the driver */

  if (irq_attach(priv_tx->irq, sai_interrupt, priv_tx))
    {
      i2serr("ERROR: Can't initialize sai irq\n");
    }
  else
    {
      up_enable_irq(priv_tx->irq);
    }

  leave_critical_section(flags);

  return (priv_rx->master) ? &priv_rx->dev : &priv_tx->dev;
}

#endif /* CONFIG_STM32H7_SAI_DRIVER_DUPLEX */
