/* ###################################################################
**     Filename    : Events.h
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
** @file Events.h
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "fsl_device_registers.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "osa1.h"
#include "us_gpio.h"
#include "sensor_spi.h"
#include "debug_console.h"
#include "isense_adc.h"
#include "drvA_pwm.h"
#include "drvB_pwm.h"
#include "drv_gpio.h"
#include "ble_uart.h"
#include "endstop_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif 


void ble_uart_RxCallback(uint32_t instance, void * uartState);

void ble_uart_TxCallback(uint32_t instance, void * uartState);

void FTM0_IRQHandler(void);

void FTM2_IRQHandler(void);

/*! isense_adc IRQ handler */
void ADC0_IRQHandler(void);

/*! sensor_spi IRQ handler */
void SPI1_IRQHandler(void);

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
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
