/**
  EXT_INT Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    ext_int.c

  @Summary
    This is the generated driver implementation file for the EXT_INT 
    driver using MPLAB® Code Configurator

  @Description:
    This source file provides implementations for driver APIs for EXT_INT. 
    Generation Information : 
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC24FJ128GA310
        Driver Version    :  1.0.0
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/**
   Section: Includes
 */
#include <xc.h>
#include "ext_int.h"
//***User Area Begin->code: Add External Interrupt handler specific headers 

//***User Area End->code: Add External Interrupt handler specific headers

/**
   Section: External Interrupt Handlers
 */

/**
  Interrupt Handler for EX_INT1 - INT1
 */
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    //***User Area Begin->code: INT1 - External Interrupt 1***

    //***User Area End->code: INT1 - External Interrupt 1***
    EX_INT1_InterruptFlagClear();
}

/**
  Interrupt Handler for EX_INT2 - INT2
 */
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    //***User Area Begin->code: INT2 - External Interrupt 2***

    //***User Area End->code: INT2 - External Interrupt 2***
    EX_INT2_InterruptFlagClear();
}
/**
    Section: External Interrupt Initializers
 */

/**
    void EXT_INT_Initialize(void)

    Initializer for the following external interrupts
 */
void EXT_INT_Initialize(void) {
}

/**
    void DRV_EXT_INT_Initialize(void)
 */
void DRV_EXT_INT_Initialize(void) {
    EXT_INT_Initialize();
}
