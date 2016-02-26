/**
  System Interrupts Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.c

  @Summary:
    This is the generated manager file for the MPLAB® Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for MPLAB® Code Configurator interrupts.
    Generation Information : 
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC24FJ128GA310
        Version           :  1.02
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB             :  MPLAB X v2.35 or v3.00
 */
/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

/**
    Section: Includes
 */
#include <xc.h>
#include "pin_manager.h"

/**
    void PIN_MANAGER_Initialize(void)
 */
void PIN_MANAGER_Initialize(void) {
    /****************************************************************************
     * Setting the GPIO of PORTA
     ***************************************************************************/
    LATA = 0x00;
    TRISA = 0xC6FF;
    /****************************************************************************
     * Setting the GPIO of PORTB
     ***************************************************************************/
    LATB = 0x00;
    TRISB = 0xFFFF;
    /****************************************************************************
     * Setting the GPIO of PORTC
     ***************************************************************************/
    LATC = 0x00;
    TRISC = 0x901E;
    /****************************************************************************
     * Setting the GPIO of PORTD
     ***************************************************************************/
    LATD = 0x00;
    TRISD = 0x77CF;
    /****************************************************************************
     * Setting the GPIO of PORTE
     ***************************************************************************/
    LATE = 0x00;
    TRISE = 0x03FF;
    /****************************************************************************
     * Setting the GPIO of PORTF
     ***************************************************************************/
    LATF = 0x00;
    TRISF = 0x31FB;
    /****************************************************************************
     * Setting the GPIO of PORTG
     ***************************************************************************/
    LATG = 0x00;
    TRISG = 0xF04F;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR
     ***************************************************************************/
    ANSA = 0x00;
    ANSB = 0xF00F;
    ANSC = 0x10;
    ANSD = 0xC0;
    ANSE = 0xF0;
    ANSG = 0x40;

    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR0bits.INT1R = 0x17; // RD2->EXT_INT:INT1
    RPINR1bits.INT2R = 0x16; // RD3->EXT_INT:INT2
    RPOR12bits.RP25R = 0x12; // RD4->OC1:OC1
    RPOR10bits.RP20R = 0x13; // RD5->OC2:OC2
    RPOR6bits.RP12R = 0x03; // RD11->UART1:U1TX
    RPINR18bits.U1RXR = 0x2A; // RD12->UART1:U1RX
    RPINR19bits.U2RXR = 0x2B; // RD14->UART2:U2RX
    RPOR2bits.RP5R = 0x05; // RD15->UART2:U2TX
    RPINR7bits.IC1R = 0x22; // RE9->IC1:IC1
    RPOR15bits.RP30R = 0x07; // RF2->SPI1:SDO1
    RPINR20bits.SDI1R = 0x10; // RF3->SPI1:SDI1
    RPOR8bits.RP17R = 0x08; // RF5->SPI1:SCK1OUT
    RPOR13bits.RP26R = 0x14; // RG7->OC3:OC3
    RPOR9bits.RP19R = 0x15; // RG8->OC4:OC4
    RPOR13bits.RP27R = 0x16; // RG9->OC5:OC5
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
}
