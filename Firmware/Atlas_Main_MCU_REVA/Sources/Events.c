/* ###################################################################
**     Filename    : Events.c
**     Project     : Atlas_Main_MCU_REVA
**     Processor   : MK22FN512VDC12
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-11-14, 12:33, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file Events.c
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         
/* MODULE Events */

#include "Cpu.h"
#include "Events.h"

#ifdef __cplusplus
extern "C" {
#endif 


/* User includes (#include below this line is not maintained by Processor Expert) */

void ble_uart_RxCallback(uint32_t instance, void * uartState)
{
  /* Write your code here ... */
}

void ble_uart_TxCallback(uint32_t instance, void * uartState)
{
  /* Write your code here ... */
}

void FTM0_IRQHandler(void)
{
  FTM_DRV_IRQHandler(FSL_DRVB_PWM);
  /* Write your code here ... */
}

void FTM2_IRQHandler(void)
{
  FTM_DRV_IRQHandler(FSL_DRVA_PWM);
  /* Write your code here ... */
}

/*! isense_adc IRQ handler */
void ADC0_IRQHandler(void)
{
  /* Write your code here ... */
}

/*! sensor_spi IRQ handler */
void SPI1_IRQHandler(void)
{
#if SENSOR_SPI_DMA_MODE
  DSPI_DRV_EdmaIRQHandler(FSL_SENSOR_SPI);
#else
  DSPI_DRV_IRQHandler(FSL_SENSOR_SPI);
#endif
  /* Write your code here ... */
}

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
