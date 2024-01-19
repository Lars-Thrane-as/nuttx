
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#ifdef CONFIG_STM32H7_VREFBUF

/* Include chip-specific clocking initialization logic */

#if defined(CONFIG_STM32H7_STM32H72X73X)
#  include "stm32h72x73x_vrefbuf.c"
#else
#  error "Unsupported STM32 H7 chip"
#endif

#include "stm32_vrefbuf.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_vrefbuf_initialize
 *
 * Description:
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) or error value
 * 
 ****************************************************************************/

int stm32_vrefbuf_initialize(void)
{
    return vrefbuf_initialize();
}

#endif
