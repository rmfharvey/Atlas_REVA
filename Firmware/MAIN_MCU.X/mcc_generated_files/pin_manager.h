/**
  System Interrupts Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

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

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
 */
#include <xc.h>
/**
    Section: Device Pin Macros
 */
/**
  @Summary
    Sets the GPIO pin, RA5, high using LATA5.

  @Description
    Sets the GPIO pin, RA5, high using LATA5.

  @Preconditions
    The RA5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA5 high (1)
    IO_RA5_SetHigh();
    </code>

 */
#define IO_RA5_SetHigh()          _LATA5 = 1
/**
  @Summary
    Sets the GPIO pin, RA5, low using LATA5.

  @Description
    Sets the GPIO pin, RA5, low using LATA5.

  @Preconditions
    The RA5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA5 low (0)
    IO_RA5_SetLow();
    </code>

 */
#define IO_RA5_SetLow()           _LATA5 = 0
/**
  @Summary
    Toggles the GPIO pin, RA5, using LATA5.

  @Description
    Toggles the GPIO pin, RA5, using LATA5.

  @Preconditions
    The RA5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA5
    IO_RA5_SetToggle();
    </code>

 */
#define IO_RA5_Toggle()           _LATA5 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA5.

  @Description
    Reads the value of the GPIO pin, RA5.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA5
    postValue = IO_RA5_GetValue();
    </code>

 */
#define IO_RA5_GetValue()         _RA5
/**
  @Summary
    Configures the GPIO pin, RA5, as an input.

  @Description
    Configures the GPIO pin, RA5, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA5 as an input
    IO_RA5_SetDigitalInput();
    </code>

 */
#define IO_RA5_SetDigitalInput()  _TRISA5 = 1
/**
  @Summary
    Configures the GPIO pin, RA5, as an output.

  @Description
    Configures the GPIO pin, RA5, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA5 as an output
    IO_RA5_SetDigitalOutput();
    </code>

 */
#define IO_RA5_SetDigitalOutput() _TRISA5 = 0
/**
  @Summary
    Sets the GPIO pin, RA6, high using LATA6.

  @Description
    Sets the GPIO pin, RA6, high using LATA6.

  @Preconditions
    The RA6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA6 high (1)
    IO_RA6_SetHigh();
    </code>

 */
#define IO_RA6_SetHigh()          _LATA6 = 1
/**
  @Summary
    Sets the GPIO pin, RA6, low using LATA6.

  @Description
    Sets the GPIO pin, RA6, low using LATA6.

  @Preconditions
    The RA6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA6 low (0)
    IO_RA6_SetLow();
    </code>

 */
#define IO_RA6_SetLow()           _LATA6 = 0
/**
  @Summary
    Toggles the GPIO pin, RA6, using LATA6.

  @Description
    Toggles the GPIO pin, RA6, using LATA6.

  @Preconditions
    The RA6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA6
    IO_RA6_SetToggle();
    </code>

 */
#define IO_RA6_Toggle()           _LATA6 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA6.

  @Description
    Reads the value of the GPIO pin, RA6.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA6
    postValue = IO_RA6_GetValue();
    </code>

 */
#define IO_RA6_GetValue()         _RA6
/**
  @Summary
    Configures the GPIO pin, RA6, as an input.

  @Description
    Configures the GPIO pin, RA6, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA6 as an input
    IO_RA6_SetDigitalInput();
    </code>

 */
#define IO_RA6_SetDigitalInput()  _TRISA6 = 1
/**
  @Summary
    Configures the GPIO pin, RA6, as an output.

  @Description
    Configures the GPIO pin, RA6, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA6 as an output
    IO_RA6_SetDigitalOutput();
    </code>

 */
#define IO_RA6_SetDigitalOutput() _TRISA6 = 0
/**
  @Summary
    Sets the GPIO pin, RA7, high using LATA7.

  @Description
    Sets the GPIO pin, RA7, high using LATA7.

  @Preconditions
    The RA7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA7 high (1)
    IO_RA7_SetHigh();
    </code>

 */
#define IO_RA7_SetHigh()          _LATA7 = 1
/**
  @Summary
    Sets the GPIO pin, RA7, low using LATA7.

  @Description
    Sets the GPIO pin, RA7, low using LATA7.

  @Preconditions
    The RA7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA7 low (0)
    IO_RA7_SetLow();
    </code>

 */
#define IO_RA7_SetLow()           _LATA7 = 0
/**
  @Summary
    Toggles the GPIO pin, RA7, using LATA7.

  @Description
    Toggles the GPIO pin, RA7, using LATA7.

  @Preconditions
    The RA7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA7
    IO_RA7_SetToggle();
    </code>

 */
#define IO_RA7_Toggle()           _LATA7 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA7.

  @Description
    Reads the value of the GPIO pin, RA7.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA7
    postValue = IO_RA7_GetValue();
    </code>

 */
#define IO_RA7_GetValue()         _RA7
/**
  @Summary
    Configures the GPIO pin, RA7, as an input.

  @Description
    Configures the GPIO pin, RA7, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA7 as an input
    IO_RA7_SetDigitalInput();
    </code>

 */
#define IO_RA7_SetDigitalInput()  _TRISA7 = 1
/**
  @Summary
    Configures the GPIO pin, RA7, as an output.

  @Description
    Configures the GPIO pin, RA7, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA7 as an output
    IO_RA7_SetDigitalOutput();
    </code>

 */
#define IO_RA7_SetDigitalOutput() _TRISA7 = 0
/**
  @Summary
    Sets the GPIO pin, RA9, high using LATA9.

  @Description
    Sets the GPIO pin, RA9, high using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA9 high (1)
    IO_RA9_SetHigh();
    </code>

 */
#define IO_RA9_SetHigh()          _LATA9 = 1
/**
  @Summary
    Sets the GPIO pin, RA9, low using LATA9.

  @Description
    Sets the GPIO pin, RA9, low using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA9 low (0)
    IO_RA9_SetLow();
    </code>

 */
#define IO_RA9_SetLow()           _LATA9 = 0
/**
  @Summary
    Toggles the GPIO pin, RA9, using LATA9.

  @Description
    Toggles the GPIO pin, RA9, using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA9
    IO_RA9_SetToggle();
    </code>

 */
#define IO_RA9_Toggle()           _LATA9 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA9.

  @Description
    Reads the value of the GPIO pin, RA9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA9
    postValue = IO_RA9_GetValue();
    </code>

 */
#define IO_RA9_GetValue()         _RA9
/**
  @Summary
    Configures the GPIO pin, RA9, as an input.

  @Description
    Configures the GPIO pin, RA9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA9 as an input
    IO_RA9_SetDigitalInput();
    </code>

 */
#define IO_RA9_SetDigitalInput()  _TRISA9 = 1
/**
  @Summary
    Configures the GPIO pin, RA9, as an output.

  @Description
    Configures the GPIO pin, RA9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA9 as an output
    IO_RA9_SetDigitalOutput();
    </code>

 */
#define IO_RA9_SetDigitalOutput() _TRISA9 = 0
/**
  @Summary
    Sets the GPIO pin, RA10, high using LATA10.

  @Description
    Sets the GPIO pin, RA10, high using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 high (1)
    IO_RA10_SetHigh();
    </code>

 */
#define IO_RA10_SetHigh()          _LATA10 = 1
/**
  @Summary
    Sets the GPIO pin, RA10, low using LATA10.

  @Description
    Sets the GPIO pin, RA10, low using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 low (0)
    IO_RA10_SetLow();
    </code>

 */
#define IO_RA10_SetLow()           _LATA10 = 0
/**
  @Summary
    Toggles the GPIO pin, RA10, using LATA10.

  @Description
    Toggles the GPIO pin, RA10, using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA10
    IO_RA10_SetToggle();
    </code>

 */
#define IO_RA10_Toggle()           _LATA10 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA10.

  @Description
    Reads the value of the GPIO pin, RA10.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA10
    postValue = IO_RA10_GetValue();
    </code>

 */
#define IO_RA10_GetValue()         _RA10
/**
  @Summary
    Configures the GPIO pin, RA10, as an input.

  @Description
    Configures the GPIO pin, RA10, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an input
    IO_RA10_SetDigitalInput();
    </code>

 */
#define IO_RA10_SetDigitalInput()  _TRISA10 = 1
/**
  @Summary
    Configures the GPIO pin, RA10, as an output.

  @Description
    Configures the GPIO pin, RA10, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an output
    IO_RA10_SetDigitalOutput();
    </code>

 */
#define IO_RA10_SetDigitalOutput() _TRISA10 = 0
/**
  @Summary
    Sets the GPIO pin, RB2, high using LATB2.

  @Description
    Sets the GPIO pin, RB2, high using LATB2.

  @Preconditions
    The RB2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB2 high (1)
    Channel_AN2_SetHigh();
    </code>

 */
#define Channel_AN2_SetHigh()          _LATB2 = 1
/**
  @Summary
    Sets the GPIO pin, RB2, low using LATB2.

  @Description
    Sets the GPIO pin, RB2, low using LATB2.

  @Preconditions
    The RB2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB2 low (0)
    Channel_AN2_SetLow();
    </code>

 */
#define Channel_AN2_SetLow()           _LATB2 = 0
/**
  @Summary
    Toggles the GPIO pin, RB2, using LATB2.

  @Description
    Toggles the GPIO pin, RB2, using LATB2.

  @Preconditions
    The RB2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB2
    Channel_AN2_SetToggle();
    </code>

 */
#define Channel_AN2_Toggle()           _LATB2 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB2.

  @Description
    Reads the value of the GPIO pin, RB2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB2
    postValue = Channel_AN2_GetValue();
    </code>

 */
#define Channel_AN2_GetValue()         _RB2
/**
  @Summary
    Configures the GPIO pin, RB2, as an input.

  @Description
    Configures the GPIO pin, RB2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB2 as an input
    Channel_AN2_SetDigitalInput();
    </code>

 */
#define Channel_AN2_SetDigitalInput()  _TRISB2 = 1
/**
  @Summary
    Configures the GPIO pin, RB2, as an output.

  @Description
    Configures the GPIO pin, RB2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB2 as an output
    Channel_AN2_SetDigitalOutput();
    </code>

 */
#define Channel_AN2_SetDigitalOutput() _TRISB2 = 0
/**
  @Summary
    Sets the GPIO pin, RB3, high using LATB3.

  @Description
    Sets the GPIO pin, RB3, high using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB3 high (1)
    Channel_AN3_SetHigh();
    </code>

 */
#define Channel_AN3_SetHigh()          _LATB3 = 1
/**
  @Summary
    Sets the GPIO pin, RB3, low using LATB3.

  @Description
    Sets the GPIO pin, RB3, low using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB3 low (0)
    Channel_AN3_SetLow();
    </code>

 */
#define Channel_AN3_SetLow()           _LATB3 = 0
/**
  @Summary
    Toggles the GPIO pin, RB3, using LATB3.

  @Description
    Toggles the GPIO pin, RB3, using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB3
    Channel_AN3_SetToggle();
    </code>

 */
#define Channel_AN3_Toggle()           _LATB3 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB3.

  @Description
    Reads the value of the GPIO pin, RB3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB3
    postValue = Channel_AN3_GetValue();
    </code>

 */
#define Channel_AN3_GetValue()         _RB3
/**
  @Summary
    Configures the GPIO pin, RB3, as an input.

  @Description
    Configures the GPIO pin, RB3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB3 as an input
    Channel_AN3_SetDigitalInput();
    </code>

 */
#define Channel_AN3_SetDigitalInput()  _TRISB3 = 1
/**
  @Summary
    Configures the GPIO pin, RB3, as an output.

  @Description
    Configures the GPIO pin, RB3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB3 as an output
    Channel_AN3_SetDigitalOutput();
    </code>

 */
#define Channel_AN3_SetDigitalOutput() _TRISB3 = 0
/**
  @Summary
    Sets the GPIO pin, RB4, high using LATB4.

  @Description
    Sets the GPIO pin, RB4, high using LATB4.

  @Preconditions
    The RB4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB4 high (1)
    IO_RB4_SetHigh();
    </code>

 */
#define IO_RB4_SetHigh()          _LATB4 = 1
/**
  @Summary
    Sets the GPIO pin, RB4, low using LATB4.

  @Description
    Sets the GPIO pin, RB4, low using LATB4.

  @Preconditions
    The RB4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB4 low (0)
    IO_RB4_SetLow();
    </code>

 */
#define IO_RB4_SetLow()           _LATB4 = 0
/**
  @Summary
    Toggles the GPIO pin, RB4, using LATB4.

  @Description
    Toggles the GPIO pin, RB4, using LATB4.

  @Preconditions
    The RB4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB4
    IO_RB4_SetToggle();
    </code>

 */
#define IO_RB4_Toggle()           _LATB4 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB4.

  @Description
    Reads the value of the GPIO pin, RB4.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB4
    postValue = IO_RB4_GetValue();
    </code>

 */
#define IO_RB4_GetValue()         _RB4
/**
  @Summary
    Configures the GPIO pin, RB4, as an input.

  @Description
    Configures the GPIO pin, RB4, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB4 as an input
    IO_RB4_SetDigitalInput();
    </code>

 */
#define IO_RB4_SetDigitalInput()  _TRISB4 = 1
/**
  @Summary
    Configures the GPIO pin, RB4, as an output.

  @Description
    Configures the GPIO pin, RB4, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB4 as an output
    IO_RB4_SetDigitalOutput();
    </code>

 */
#define IO_RB4_SetDigitalOutput() _TRISB4 = 0
/**
  @Summary
    Sets the GPIO pin, RB5, high using LATB5.

  @Description
    Sets the GPIO pin, RB5, high using LATB5.

  @Preconditions
    The RB5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB5 high (1)
    IO_RB5_SetHigh();
    </code>

 */
#define IO_RB5_SetHigh()          _LATB5 = 1
/**
  @Summary
    Sets the GPIO pin, RB5, low using LATB5.

  @Description
    Sets the GPIO pin, RB5, low using LATB5.

  @Preconditions
    The RB5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB5 low (0)
    IO_RB5_SetLow();
    </code>

 */
#define IO_RB5_SetLow()           _LATB5 = 0
/**
  @Summary
    Toggles the GPIO pin, RB5, using LATB5.

  @Description
    Toggles the GPIO pin, RB5, using LATB5.

  @Preconditions
    The RB5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB5
    IO_RB5_SetToggle();
    </code>

 */
#define IO_RB5_Toggle()           _LATB5 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB5.

  @Description
    Reads the value of the GPIO pin, RB5.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB5
    postValue = IO_RB5_GetValue();
    </code>

 */
#define IO_RB5_GetValue()         _RB5
/**
  @Summary
    Configures the GPIO pin, RB5, as an input.

  @Description
    Configures the GPIO pin, RB5, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB5 as an input
    IO_RB5_SetDigitalInput();
    </code>

 */
#define IO_RB5_SetDigitalInput()  _TRISB5 = 1
/**
  @Summary
    Configures the GPIO pin, RB5, as an output.

  @Description
    Configures the GPIO pin, RB5, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB5 as an output
    IO_RB5_SetDigitalOutput();
    </code>

 */
#define IO_RB5_SetDigitalOutput() _TRISB5 = 0
/**
  @Summary
    Sets the GPIO pin, RB6, high using LATB6.

  @Description
    Sets the GPIO pin, RB6, high using LATB6.

  @Preconditions
    The RB6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB6 high (1)
    IO_RB6_SetHigh();
    </code>

 */
#define IO_RB6_SetHigh()          _LATB6 = 1
/**
  @Summary
    Sets the GPIO pin, RB6, low using LATB6.

  @Description
    Sets the GPIO pin, RB6, low using LATB6.

  @Preconditions
    The RB6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB6 low (0)
    IO_RB6_SetLow();
    </code>

 */
#define IO_RB6_SetLow()           _LATB6 = 0
/**
  @Summary
    Toggles the GPIO pin, RB6, using LATB6.

  @Description
    Toggles the GPIO pin, RB6, using LATB6.

  @Preconditions
    The RB6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB6
    IO_RB6_SetToggle();
    </code>

 */
#define IO_RB6_Toggle()           _LATB6 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB6.

  @Description
    Reads the value of the GPIO pin, RB6.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB6
    postValue = IO_RB6_GetValue();
    </code>

 */
#define IO_RB6_GetValue()         _RB6
/**
  @Summary
    Configures the GPIO pin, RB6, as an input.

  @Description
    Configures the GPIO pin, RB6, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB6 as an input
    IO_RB6_SetDigitalInput();
    </code>

 */
#define IO_RB6_SetDigitalInput()  _TRISB6 = 1
/**
  @Summary
    Configures the GPIO pin, RB6, as an output.

  @Description
    Configures the GPIO pin, RB6, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB6 as an output
    IO_RB6_SetDigitalOutput();
    </code>

 */
#define IO_RB6_SetDigitalOutput() _TRISB6 = 0
/**
  @Summary
    Sets the GPIO pin, RB7, high using LATB7.

  @Description
    Sets the GPIO pin, RB7, high using LATB7.

  @Preconditions
    The RB7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB7 high (1)
    IO_RB7_SetHigh();
    </code>

 */
#define IO_RB7_SetHigh()          _LATB7 = 1
/**
  @Summary
    Sets the GPIO pin, RB7, low using LATB7.

  @Description
    Sets the GPIO pin, RB7, low using LATB7.

  @Preconditions
    The RB7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB7 low (0)
    IO_RB7_SetLow();
    </code>

 */
#define IO_RB7_SetLow()           _LATB7 = 0
/**
  @Summary
    Toggles the GPIO pin, RB7, using LATB7.

  @Description
    Toggles the GPIO pin, RB7, using LATB7.

  @Preconditions
    The RB7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB7
    IO_RB7_SetToggle();
    </code>

 */
#define IO_RB7_Toggle()           _LATB7 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB7.

  @Description
    Reads the value of the GPIO pin, RB7.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB7
    postValue = IO_RB7_GetValue();
    </code>

 */
#define IO_RB7_GetValue()         _RB7
/**
  @Summary
    Configures the GPIO pin, RB7, as an input.

  @Description
    Configures the GPIO pin, RB7, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB7 as an input
    IO_RB7_SetDigitalInput();
    </code>

 */
#define IO_RB7_SetDigitalInput()  _TRISB7 = 1
/**
  @Summary
    Configures the GPIO pin, RB7, as an output.

  @Description
    Configures the GPIO pin, RB7, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB7 as an output
    IO_RB7_SetDigitalOutput();
    </code>

 */
#define IO_RB7_SetDigitalOutput() _TRISB7 = 0
/**
  @Summary
    Sets the GPIO pin, RB8, high using LATB8.

  @Description
    Sets the GPIO pin, RB8, high using LATB8.

  @Preconditions
    The RB8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB8 high (1)
    IO_RB8_SetHigh();
    </code>

 */
#define IO_RB8_SetHigh()          _LATB8 = 1
/**
  @Summary
    Sets the GPIO pin, RB8, low using LATB8.

  @Description
    Sets the GPIO pin, RB8, low using LATB8.

  @Preconditions
    The RB8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB8 low (0)
    IO_RB8_SetLow();
    </code>

 */
#define IO_RB8_SetLow()           _LATB8 = 0
/**
  @Summary
    Toggles the GPIO pin, RB8, using LATB8.

  @Description
    Toggles the GPIO pin, RB8, using LATB8.

  @Preconditions
    The RB8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB8
    IO_RB8_SetToggle();
    </code>

 */
#define IO_RB8_Toggle()           _LATB8 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB8.

  @Description
    Reads the value of the GPIO pin, RB8.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB8
    postValue = IO_RB8_GetValue();
    </code>

 */
#define IO_RB8_GetValue()         _RB8
/**
  @Summary
    Configures the GPIO pin, RB8, as an input.

  @Description
    Configures the GPIO pin, RB8, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB8 as an input
    IO_RB8_SetDigitalInput();
    </code>

 */
#define IO_RB8_SetDigitalInput()  _TRISB8 = 1
/**
  @Summary
    Configures the GPIO pin, RB8, as an output.

  @Description
    Configures the GPIO pin, RB8, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB8 as an output
    IO_RB8_SetDigitalOutput();
    </code>

 */
#define IO_RB8_SetDigitalOutput() _TRISB8 = 0
/**
  @Summary
    Sets the GPIO pin, RB9, high using LATB9.

  @Description
    Sets the GPIO pin, RB9, high using LATB9.

  @Preconditions
    The RB9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB9 high (1)
    IO_RB9_SetHigh();
    </code>

 */
#define IO_RB9_SetHigh()          _LATB9 = 1
/**
  @Summary
    Sets the GPIO pin, RB9, low using LATB9.

  @Description
    Sets the GPIO pin, RB9, low using LATB9.

  @Preconditions
    The RB9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB9 low (0)
    IO_RB9_SetLow();
    </code>

 */
#define IO_RB9_SetLow()           _LATB9 = 0
/**
  @Summary
    Toggles the GPIO pin, RB9, using LATB9.

  @Description
    Toggles the GPIO pin, RB9, using LATB9.

  @Preconditions
    The RB9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB9
    IO_RB9_SetToggle();
    </code>

 */
#define IO_RB9_Toggle()           _LATB9 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB9.

  @Description
    Reads the value of the GPIO pin, RB9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB9
    postValue = IO_RB9_GetValue();
    </code>

 */
#define IO_RB9_GetValue()         _RB9
/**
  @Summary
    Configures the GPIO pin, RB9, as an input.

  @Description
    Configures the GPIO pin, RB9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB9 as an input
    IO_RB9_SetDigitalInput();
    </code>

 */
#define IO_RB9_SetDigitalInput()  _TRISB9 = 1
/**
  @Summary
    Configures the GPIO pin, RB9, as an output.

  @Description
    Configures the GPIO pin, RB9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB9 as an output
    IO_RB9_SetDigitalOutput();
    </code>

 */
#define IO_RB9_SetDigitalOutput() _TRISB9 = 0
/**
  @Summary
    Sets the GPIO pin, RB10, high using LATB10.

  @Description
    Sets the GPIO pin, RB10, high using LATB10.

  @Preconditions
    The RB10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB10 high (1)
    IO_RB10_SetHigh();
    </code>

 */
#define IO_RB10_SetHigh()          _LATB10 = 1
/**
  @Summary
    Sets the GPIO pin, RB10, low using LATB10.

  @Description
    Sets the GPIO pin, RB10, low using LATB10.

  @Preconditions
    The RB10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB10 low (0)
    IO_RB10_SetLow();
    </code>

 */
#define IO_RB10_SetLow()           _LATB10 = 0
/**
  @Summary
    Toggles the GPIO pin, RB10, using LATB10.

  @Description
    Toggles the GPIO pin, RB10, using LATB10.

  @Preconditions
    The RB10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB10
    IO_RB10_SetToggle();
    </code>

 */
#define IO_RB10_Toggle()           _LATB10 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB10.

  @Description
    Reads the value of the GPIO pin, RB10.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB10
    postValue = IO_RB10_GetValue();
    </code>

 */
#define IO_RB10_GetValue()         _RB10
/**
  @Summary
    Configures the GPIO pin, RB10, as an input.

  @Description
    Configures the GPIO pin, RB10, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB10 as an input
    IO_RB10_SetDigitalInput();
    </code>

 */
#define IO_RB10_SetDigitalInput()  _TRISB10 = 1
/**
  @Summary
    Configures the GPIO pin, RB10, as an output.

  @Description
    Configures the GPIO pin, RB10, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB10 as an output
    IO_RB10_SetDigitalOutput();
    </code>

 */
#define IO_RB10_SetDigitalOutput() _TRISB10 = 0
/**
  @Summary
    Sets the GPIO pin, RB11, high using LATB11.

  @Description
    Sets the GPIO pin, RB11, high using LATB11.

  @Preconditions
    The RB11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB11 high (1)
    IO_RB11_SetHigh();
    </code>

 */
#define IO_RB11_SetHigh()          _LATB11 = 1
/**
  @Summary
    Sets the GPIO pin, RB11, low using LATB11.

  @Description
    Sets the GPIO pin, RB11, low using LATB11.

  @Preconditions
    The RB11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB11 low (0)
    IO_RB11_SetLow();
    </code>

 */
#define IO_RB11_SetLow()           _LATB11 = 0
/**
  @Summary
    Toggles the GPIO pin, RB11, using LATB11.

  @Description
    Toggles the GPIO pin, RB11, using LATB11.

  @Preconditions
    The RB11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB11
    IO_RB11_SetToggle();
    </code>

 */
#define IO_RB11_Toggle()           _LATB11 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB11.

  @Description
    Reads the value of the GPIO pin, RB11.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB11
    postValue = IO_RB11_GetValue();
    </code>

 */
#define IO_RB11_GetValue()         _RB11
/**
  @Summary
    Configures the GPIO pin, RB11, as an input.

  @Description
    Configures the GPIO pin, RB11, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB11 as an input
    IO_RB11_SetDigitalInput();
    </code>

 */
#define IO_RB11_SetDigitalInput()  _TRISB11 = 1
/**
  @Summary
    Configures the GPIO pin, RB11, as an output.

  @Description
    Configures the GPIO pin, RB11, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB11 as an output
    IO_RB11_SetDigitalOutput();
    </code>

 */
#define IO_RB11_SetDigitalOutput() _TRISB11 = 0
/**
  @Summary
    Sets the GPIO pin, RC1, high using LATC1.

  @Description
    Sets the GPIO pin, RC1, high using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 high (1)
    IO_RC1_SetHigh();
    </code>

 */
#define IO_RC1_SetHigh()          _LATC1 = 1
/**
  @Summary
    Sets the GPIO pin, RC1, low using LATC1.

  @Description
    Sets the GPIO pin, RC1, low using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 low (0)
    IO_RC1_SetLow();
    </code>

 */
#define IO_RC1_SetLow()           _LATC1 = 0
/**
  @Summary
    Toggles the GPIO pin, RC1, using LATC1.

  @Description
    Toggles the GPIO pin, RC1, using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC1
    IO_RC1_SetToggle();
    </code>

 */
#define IO_RC1_Toggle()           _LATC1 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC1.

  @Description
    Reads the value of the GPIO pin, RC1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC1
    postValue = IO_RC1_GetValue();
    </code>

 */
#define IO_RC1_GetValue()         _RC1
/**
  @Summary
    Configures the GPIO pin, RC1, as an input.

  @Description
    Configures the GPIO pin, RC1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an input
    IO_RC1_SetDigitalInput();
    </code>

 */
#define IO_RC1_SetDigitalInput()  _TRISC1 = 1
/**
  @Summary
    Configures the GPIO pin, RC1, as an output.

  @Description
    Configures the GPIO pin, RC1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an output
    IO_RC1_SetDigitalOutput();
    </code>

 */
#define IO_RC1_SetDigitalOutput() _TRISC1 = 0
/**
  @Summary
    Sets the GPIO pin, RC2, high using LATC2.

  @Description
    Sets the GPIO pin, RC2, high using LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC2 high (1)
    IO_RC2_SetHigh();
    </code>

 */
#define IO_RC2_SetHigh()          _LATC2 = 1
/**
  @Summary
    Sets the GPIO pin, RC2, low using LATC2.

  @Description
    Sets the GPIO pin, RC2, low using LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC2 low (0)
    IO_RC2_SetLow();
    </code>

 */
#define IO_RC2_SetLow()           _LATC2 = 0
/**
  @Summary
    Toggles the GPIO pin, RC2, using LATC2.

  @Description
    Toggles the GPIO pin, RC2, using LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC2
    IO_RC2_SetToggle();
    </code>

 */
#define IO_RC2_Toggle()           _LATC2 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC2.

  @Description
    Reads the value of the GPIO pin, RC2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC2
    postValue = IO_RC2_GetValue();
    </code>

 */
#define IO_RC2_GetValue()         _RC2
/**
  @Summary
    Configures the GPIO pin, RC2, as an input.

  @Description
    Configures the GPIO pin, RC2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC2 as an input
    IO_RC2_SetDigitalInput();
    </code>

 */
#define IO_RC2_SetDigitalInput()  _TRISC2 = 1
/**
  @Summary
    Configures the GPIO pin, RC2, as an output.

  @Description
    Configures the GPIO pin, RC2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC2 as an output
    IO_RC2_SetDigitalOutput();
    </code>

 */
#define IO_RC2_SetDigitalOutput() _TRISC2 = 0
/**
  @Summary
    Sets the GPIO pin, RD2, high using LATD2.

  @Description
    Sets the GPIO pin, RD2, high using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD2 high (1)
    EX_INT1_SetHigh();
    </code>

 */
#define EX_INT1_SetHigh()          _LATD2 = 1
/**
  @Summary
    Sets the GPIO pin, RD2, low using LATD2.

  @Description
    Sets the GPIO pin, RD2, low using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD2 low (0)
    EX_INT1_SetLow();
    </code>

 */
#define EX_INT1_SetLow()           _LATD2 = 0
/**
  @Summary
    Toggles the GPIO pin, RD2, using LATD2.

  @Description
    Toggles the GPIO pin, RD2, using LATD2.

  @Preconditions
    The RD2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD2
    EX_INT1_SetToggle();
    </code>

 */
#define EX_INT1_Toggle()           _LATD2 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD2.

  @Description
    Reads the value of the GPIO pin, RD2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD2
    postValue = EX_INT1_GetValue();
    </code>

 */
#define EX_INT1_GetValue()         _RD2
/**
  @Summary
    Configures the GPIO pin, RD2, as an input.

  @Description
    Configures the GPIO pin, RD2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD2 as an input
    EX_INT1_SetDigitalInput();
    </code>

 */
#define EX_INT1_SetDigitalInput()  _TRISD2 = 1
/**
  @Summary
    Configures the GPIO pin, RD2, as an output.

  @Description
    Configures the GPIO pin, RD2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD2 as an output
    EX_INT1_SetDigitalOutput();
    </code>

 */
#define EX_INT1_SetDigitalOutput() _TRISD2 = 0
/**
  @Summary
    Sets the GPIO pin, RD3, high using LATD3.

  @Description
    Sets the GPIO pin, RD3, high using LATD3.

  @Preconditions
    The RD3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD3 high (1)
    EX_INT2_SetHigh();
    </code>

 */
#define EX_INT2_SetHigh()          _LATD3 = 1
/**
  @Summary
    Sets the GPIO pin, RD3, low using LATD3.

  @Description
    Sets the GPIO pin, RD3, low using LATD3.

  @Preconditions
    The RD3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD3 low (0)
    EX_INT2_SetLow();
    </code>

 */
#define EX_INT2_SetLow()           _LATD3 = 0
/**
  @Summary
    Toggles the GPIO pin, RD3, using LATD3.

  @Description
    Toggles the GPIO pin, RD3, using LATD3.

  @Preconditions
    The RD3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD3
    EX_INT2_SetToggle();
    </code>

 */
#define EX_INT2_Toggle()           _LATD3 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD3.

  @Description
    Reads the value of the GPIO pin, RD3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD3
    postValue = EX_INT2_GetValue();
    </code>

 */
#define EX_INT2_GetValue()         _RD3
/**
  @Summary
    Configures the GPIO pin, RD3, as an input.

  @Description
    Configures the GPIO pin, RD3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD3 as an input
    EX_INT2_SetDigitalInput();
    </code>

 */
#define EX_INT2_SetDigitalInput()  _TRISD3 = 1
/**
  @Summary
    Configures the GPIO pin, RD3, as an output.

  @Description
    Configures the GPIO pin, RD3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD3 as an output
    EX_INT2_SetDigitalOutput();
    </code>

 */
#define EX_INT2_SetDigitalOutput() _TRISD3 = 0
/**
  @Summary
    Sets the GPIO pin, RD9, high using LATD9.

  @Description
    Sets the GPIO pin, RD9, high using LATD9.

  @Preconditions
    The RD9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD9 high (1)
    IO_RD9_SetHigh();
    </code>

 */
#define IO_RD9_SetHigh()          _LATD9 = 1
/**
  @Summary
    Sets the GPIO pin, RD9, low using LATD9.

  @Description
    Sets the GPIO pin, RD9, low using LATD9.

  @Preconditions
    The RD9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD9 low (0)
    IO_RD9_SetLow();
    </code>

 */
#define IO_RD9_SetLow()           _LATD9 = 0
/**
  @Summary
    Toggles the GPIO pin, RD9, using LATD9.

  @Description
    Toggles the GPIO pin, RD9, using LATD9.

  @Preconditions
    The RD9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD9
    IO_RD9_SetToggle();
    </code>

 */
#define IO_RD9_Toggle()           _LATD9 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD9.

  @Description
    Reads the value of the GPIO pin, RD9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD9
    postValue = IO_RD9_GetValue();
    </code>

 */
#define IO_RD9_GetValue()         _RD9
/**
  @Summary
    Configures the GPIO pin, RD9, as an input.

  @Description
    Configures the GPIO pin, RD9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD9 as an input
    IO_RD9_SetDigitalInput();
    </code>

 */
#define IO_RD9_SetDigitalInput()  _TRISD9 = 1
/**
  @Summary
    Configures the GPIO pin, RD9, as an output.

  @Description
    Configures the GPIO pin, RD9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD9 as an output
    IO_RD9_SetDigitalOutput();
    </code>

 */
#define IO_RD9_SetDigitalOutput() _TRISD9 = 0
/**
  @Summary
    Sets the GPIO pin, RD10, high using LATD10.

  @Description
    Sets the GPIO pin, RD10, high using LATD10.

  @Preconditions
    The RD10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD10 high (1)
    IO_RD10_SetHigh();
    </code>

 */
#define IO_RD10_SetHigh()          _LATD10 = 1
/**
  @Summary
    Sets the GPIO pin, RD10, low using LATD10.

  @Description
    Sets the GPIO pin, RD10, low using LATD10.

  @Preconditions
    The RD10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD10 low (0)
    IO_RD10_SetLow();
    </code>

 */
#define IO_RD10_SetLow()           _LATD10 = 0
/**
  @Summary
    Toggles the GPIO pin, RD10, using LATD10.

  @Description
    Toggles the GPIO pin, RD10, using LATD10.

  @Preconditions
    The RD10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD10
    IO_RD10_SetToggle();
    </code>

 */
#define IO_RD10_Toggle()           _LATD10 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD10.

  @Description
    Reads the value of the GPIO pin, RD10.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD10
    postValue = IO_RD10_GetValue();
    </code>

 */
#define IO_RD10_GetValue()         _RD10
/**
  @Summary
    Configures the GPIO pin, RD10, as an input.

  @Description
    Configures the GPIO pin, RD10, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD10 as an input
    IO_RD10_SetDigitalInput();
    </code>

 */
#define IO_RD10_SetDigitalInput()  _TRISD10 = 1
/**
  @Summary
    Configures the GPIO pin, RD10, as an output.

  @Description
    Configures the GPIO pin, RD10, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD10 as an output
    IO_RD10_SetDigitalOutput();
    </code>

 */
#define IO_RD10_SetDigitalOutput() _TRISD10 = 0
/**
  @Summary
    Sets the GPIO pin, RD11, high using LATD11.

  @Description
    Sets the GPIO pin, RD11, high using LATD11.

  @Preconditions
    The RD11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD11 high (1)
    U1TX_SetHigh();
    </code>

 */
#define U1TX_SetHigh()          _LATD11 = 1
/**
  @Summary
    Sets the GPIO pin, RD11, low using LATD11.

  @Description
    Sets the GPIO pin, RD11, low using LATD11.

  @Preconditions
    The RD11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD11 low (0)
    U1TX_SetLow();
    </code>

 */
#define U1TX_SetLow()           _LATD11 = 0
/**
  @Summary
    Toggles the GPIO pin, RD11, using LATD11.

  @Description
    Toggles the GPIO pin, RD11, using LATD11.

  @Preconditions
    The RD11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD11
    U1TX_SetToggle();
    </code>

 */
#define U1TX_Toggle()           _LATD11 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD11.

  @Description
    Reads the value of the GPIO pin, RD11.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD11
    postValue = U1TX_GetValue();
    </code>

 */
#define U1TX_GetValue()         _RD11
/**
  @Summary
    Configures the GPIO pin, RD11, as an input.

  @Description
    Configures the GPIO pin, RD11, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD11 as an input
    U1TX_SetDigitalInput();
    </code>

 */
#define U1TX_SetDigitalInput()  _TRISD11 = 1
/**
  @Summary
    Configures the GPIO pin, RD11, as an output.

  @Description
    Configures the GPIO pin, RD11, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD11 as an output
    U1TX_SetDigitalOutput();
    </code>

 */
#define U1TX_SetDigitalOutput() _TRISD11 = 0
/**
  @Summary
    Sets the GPIO pin, RD12, high using LATD12.

  @Description
    Sets the GPIO pin, RD12, high using LATD12.

  @Preconditions
    The RD12 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD12 high (1)
    U1RX_SetHigh();
    </code>

 */
#define U1RX_SetHigh()          _LATD12 = 1
/**
  @Summary
    Sets the GPIO pin, RD12, low using LATD12.

  @Description
    Sets the GPIO pin, RD12, low using LATD12.

  @Preconditions
    The RD12 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD12 low (0)
    U1RX_SetLow();
    </code>

 */
#define U1RX_SetLow()           _LATD12 = 0
/**
  @Summary
    Toggles the GPIO pin, RD12, using LATD12.

  @Description
    Toggles the GPIO pin, RD12, using LATD12.

  @Preconditions
    The RD12 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD12
    U1RX_SetToggle();
    </code>

 */
#define U1RX_Toggle()           _LATD12 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD12.

  @Description
    Reads the value of the GPIO pin, RD12.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD12
    postValue = U1RX_GetValue();
    </code>

 */
#define U1RX_GetValue()         _RD12
/**
  @Summary
    Configures the GPIO pin, RD12, as an input.

  @Description
    Configures the GPIO pin, RD12, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD12 as an input
    U1RX_SetDigitalInput();
    </code>

 */
#define U1RX_SetDigitalInput()  _TRISD12 = 1
/**
  @Summary
    Configures the GPIO pin, RD12, as an output.

  @Description
    Configures the GPIO pin, RD12, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD12 as an output
    U1RX_SetDigitalOutput();
    </code>

 */
#define U1RX_SetDigitalOutput() _TRISD12 = 0
/**
  @Summary
    Sets the GPIO pin, RD13, high using LATD13.

  @Description
    Sets the GPIO pin, RD13, high using LATD13.

  @Preconditions
    The RD13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD13 high (1)
    IO_RD13_SetHigh();
    </code>

 */
#define IO_RD13_SetHigh()          _LATD13 = 1
/**
  @Summary
    Sets the GPIO pin, RD13, low using LATD13.

  @Description
    Sets the GPIO pin, RD13, low using LATD13.

  @Preconditions
    The RD13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD13 low (0)
    IO_RD13_SetLow();
    </code>

 */
#define IO_RD13_SetLow()           _LATD13 = 0
/**
  @Summary
    Toggles the GPIO pin, RD13, using LATD13.

  @Description
    Toggles the GPIO pin, RD13, using LATD13.

  @Preconditions
    The RD13 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD13
    IO_RD13_SetToggle();
    </code>

 */
#define IO_RD13_Toggle()           _LATD13 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD13.

  @Description
    Reads the value of the GPIO pin, RD13.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD13
    postValue = IO_RD13_GetValue();
    </code>

 */
#define IO_RD13_GetValue()         _RD13
/**
  @Summary
    Configures the GPIO pin, RD13, as an input.

  @Description
    Configures the GPIO pin, RD13, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD13 as an input
    IO_RD13_SetDigitalInput();
    </code>

 */
#define IO_RD13_SetDigitalInput()  _TRISD13 = 1
/**
  @Summary
    Configures the GPIO pin, RD13, as an output.

  @Description
    Configures the GPIO pin, RD13, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD13 as an output
    IO_RD13_SetDigitalOutput();
    </code>

 */
#define IO_RD13_SetDigitalOutput() _TRISD13 = 0
/**
  @Summary
    Sets the GPIO pin, RD14, high using LATD14.

  @Description
    Sets the GPIO pin, RD14, high using LATD14.

  @Preconditions
    The RD14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD14 high (1)
    U2RX_SetHigh();
    </code>

 */
#define U2RX_SetHigh()          _LATD14 = 1
/**
  @Summary
    Sets the GPIO pin, RD14, low using LATD14.

  @Description
    Sets the GPIO pin, RD14, low using LATD14.

  @Preconditions
    The RD14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD14 low (0)
    U2RX_SetLow();
    </code>

 */
#define U2RX_SetLow()           _LATD14 = 0
/**
  @Summary
    Toggles the GPIO pin, RD14, using LATD14.

  @Description
    Toggles the GPIO pin, RD14, using LATD14.

  @Preconditions
    The RD14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD14
    U2RX_SetToggle();
    </code>

 */
#define U2RX_Toggle()           _LATD14 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD14.

  @Description
    Reads the value of the GPIO pin, RD14.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD14
    postValue = U2RX_GetValue();
    </code>

 */
#define U2RX_GetValue()         _RD14
/**
  @Summary
    Configures the GPIO pin, RD14, as an input.

  @Description
    Configures the GPIO pin, RD14, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD14 as an input
    U2RX_SetDigitalInput();
    </code>

 */
#define U2RX_SetDigitalInput()  _TRISD14 = 1
/**
  @Summary
    Configures the GPIO pin, RD14, as an output.

  @Description
    Configures the GPIO pin, RD14, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD14 as an output
    U2RX_SetDigitalOutput();
    </code>

 */
#define U2RX_SetDigitalOutput() _TRISD14 = 0
/**
  @Summary
    Sets the GPIO pin, RD15, high using LATD15.

  @Description
    Sets the GPIO pin, RD15, high using LATD15.

  @Preconditions
    The RD15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD15 high (1)
    U2TX_SetHigh();
    </code>

 */
#define U2TX_SetHigh()          _LATD15 = 1
/**
  @Summary
    Sets the GPIO pin, RD15, low using LATD15.

  @Description
    Sets the GPIO pin, RD15, low using LATD15.

  @Preconditions
    The RD15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD15 low (0)
    U2TX_SetLow();
    </code>

 */
#define U2TX_SetLow()           _LATD15 = 0
/**
  @Summary
    Toggles the GPIO pin, RD15, using LATD15.

  @Description
    Toggles the GPIO pin, RD15, using LATD15.

  @Preconditions
    The RD15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD15
    U2TX_SetToggle();
    </code>

 */
#define U2TX_Toggle()           _LATD15 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD15.

  @Description
    Reads the value of the GPIO pin, RD15.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD15
    postValue = U2TX_GetValue();
    </code>

 */
#define U2TX_GetValue()         _RD15
/**
  @Summary
    Configures the GPIO pin, RD15, as an input.

  @Description
    Configures the GPIO pin, RD15, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD15 as an input
    U2TX_SetDigitalInput();
    </code>

 */
#define U2TX_SetDigitalInput()  _TRISD15 = 1
/**
  @Summary
    Configures the GPIO pin, RD15, as an output.

  @Description
    Configures the GPIO pin, RD15, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD15 as an output
    U2TX_SetDigitalOutput();
    </code>

 */
#define U2TX_SetDigitalOutput() _TRISD15 = 0
/**
  @Summary
    Sets the GPIO pin, RF4, high using LATF4.

  @Description
    Sets the GPIO pin, RF4, high using LATF4.

  @Preconditions
    The RF4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RF4 high (1)
    IO_RF4_SetHigh();
    </code>

 */
#define IO_RF4_SetHigh()          _LATF4 = 1
/**
  @Summary
    Sets the GPIO pin, RF4, low using LATF4.

  @Description
    Sets the GPIO pin, RF4, low using LATF4.

  @Preconditions
    The RF4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RF4 low (0)
    IO_RF4_SetLow();
    </code>

 */
#define IO_RF4_SetLow()           _LATF4 = 0
/**
  @Summary
    Toggles the GPIO pin, RF4, using LATF4.

  @Description
    Toggles the GPIO pin, RF4, using LATF4.

  @Preconditions
    The RF4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RF4
    IO_RF4_SetToggle();
    </code>

 */
#define IO_RF4_Toggle()           _LATF4 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RF4.

  @Description
    Reads the value of the GPIO pin, RF4.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RF4
    postValue = IO_RF4_GetValue();
    </code>

 */
#define IO_RF4_GetValue()         _RF4
/**
  @Summary
    Configures the GPIO pin, RF4, as an input.

  @Description
    Configures the GPIO pin, RF4, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RF4 as an input
    IO_RF4_SetDigitalInput();
    </code>

 */
#define IO_RF4_SetDigitalInput()  _TRISF4 = 1
/**
  @Summary
    Configures the GPIO pin, RF4, as an output.

  @Description
    Configures the GPIO pin, RF4, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RF4 as an output
    IO_RF4_SetDigitalOutput();
    </code>

 */
#define IO_RF4_SetDigitalOutput() _TRISF4 = 0
/**
  @Summary
    Sets the GPIO pin, RF6, high using LATF6.

  @Description
    Sets the GPIO pin, RF6, high using LATF6.

  @Preconditions
    The RF6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RF6 high (1)
    IO_RF6_SetHigh();
    </code>

 */
#define IO_RF6_SetHigh()          _LATF6 = 1
/**
  @Summary
    Sets the GPIO pin, RF6, low using LATF6.

  @Description
    Sets the GPIO pin, RF6, low using LATF6.

  @Preconditions
    The RF6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RF6 low (0)
    IO_RF6_SetLow();
    </code>

 */
#define IO_RF6_SetLow()           _LATF6 = 0
/**
  @Summary
    Toggles the GPIO pin, RF6, using LATF6.

  @Description
    Toggles the GPIO pin, RF6, using LATF6.

  @Preconditions
    The RF6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RF6
    IO_RF6_SetToggle();
    </code>

 */
#define IO_RF6_Toggle()           _LATF6 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RF6.

  @Description
    Reads the value of the GPIO pin, RF6.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RF6
    postValue = IO_RF6_GetValue();
    </code>

 */
#define IO_RF6_GetValue()         _RF6
/**
  @Summary
    Configures the GPIO pin, RF6, as an input.

  @Description
    Configures the GPIO pin, RF6, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RF6 as an input
    IO_RF6_SetDigitalInput();
    </code>

 */
#define IO_RF6_SetDigitalInput()  _TRISF6 = 1
/**
  @Summary
    Configures the GPIO pin, RF6, as an output.

  @Description
    Configures the GPIO pin, RF6, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RF6 as an output
    IO_RF6_SetDigitalOutput();
    </code>

 */
#define IO_RF6_SetDigitalOutput() _TRISF6 = 0
/**
  @Summary
    Sets the GPIO pin, RF7, high using LATF7.

  @Description
    Sets the GPIO pin, RF7, high using LATF7.

  @Preconditions
    The RF7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RF7 high (1)
    IO_RF7_SetHigh();
    </code>

 */
#define IO_RF7_SetHigh()          _LATF7 = 1
/**
  @Summary
    Sets the GPIO pin, RF7, low using LATF7.

  @Description
    Sets the GPIO pin, RF7, low using LATF7.

  @Preconditions
    The RF7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RF7 low (0)
    IO_RF7_SetLow();
    </code>

 */
#define IO_RF7_SetLow()           _LATF7 = 0
/**
  @Summary
    Toggles the GPIO pin, RF7, using LATF7.

  @Description
    Toggles the GPIO pin, RF7, using LATF7.

  @Preconditions
    The RF7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RF7
    IO_RF7_SetToggle();
    </code>

 */
#define IO_RF7_Toggle()           _LATF7 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RF7.

  @Description
    Reads the value of the GPIO pin, RF7.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RF7
    postValue = IO_RF7_GetValue();
    </code>

 */
#define IO_RF7_GetValue()         _RF7
/**
  @Summary
    Configures the GPIO pin, RF7, as an input.

  @Description
    Configures the GPIO pin, RF7, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RF7 as an input
    IO_RF7_SetDigitalInput();
    </code>

 */
#define IO_RF7_SetDigitalInput()  _TRISF7 = 1
/**
  @Summary
    Configures the GPIO pin, RF7, as an output.

  @Description
    Configures the GPIO pin, RF7, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RF7 as an output
    IO_RF7_SetDigitalOutput();
    </code>

 */
#define IO_RF7_SetDigitalOutput() _TRISF7 = 0
/**
  @Summary
    Sets the GPIO pin, RG2, high using LATG2.

  @Description
    Sets the GPIO pin, RG2, high using LATG2.

  @Preconditions
    The RG2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG2 high (1)
    IO_RG2_SetHigh();
    </code>

 */
#define IO_RG2_SetHigh()          _LATG2 = 1
/**
  @Summary
    Sets the GPIO pin, RG2, low using LATG2.

  @Description
    Sets the GPIO pin, RG2, low using LATG2.

  @Preconditions
    The RG2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG2 low (0)
    IO_RG2_SetLow();
    </code>

 */
#define IO_RG2_SetLow()           _LATG2 = 0
/**
  @Summary
    Toggles the GPIO pin, RG2, using LATG2.

  @Description
    Toggles the GPIO pin, RG2, using LATG2.

  @Preconditions
    The RG2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG2
    IO_RG2_SetToggle();
    </code>

 */
#define IO_RG2_Toggle()           _LATG2 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG2.

  @Description
    Reads the value of the GPIO pin, RG2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG2
    postValue = IO_RG2_GetValue();
    </code>

 */
#define IO_RG2_GetValue()         _RG2
/**
  @Summary
    Configures the GPIO pin, RG2, as an input.

  @Description
    Configures the GPIO pin, RG2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG2 as an input
    IO_RG2_SetDigitalInput();
    </code>

 */
#define IO_RG2_SetDigitalInput()  _TRISG2 = 1
/**
  @Summary
    Configures the GPIO pin, RG2, as an output.

  @Description
    Configures the GPIO pin, RG2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG2 as an output
    IO_RG2_SetDigitalOutput();
    </code>

 */
#define IO_RG2_SetDigitalOutput() _TRISG2 = 0
/**
  @Summary
    Sets the GPIO pin, RG3, high using LATG3.

  @Description
    Sets the GPIO pin, RG3, high using LATG3.

  @Preconditions
    The RG3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG3 high (1)
    IO_RG3_SetHigh();
    </code>

 */
#define IO_RG3_SetHigh()          _LATG3 = 1
/**
  @Summary
    Sets the GPIO pin, RG3, low using LATG3.

  @Description
    Sets the GPIO pin, RG3, low using LATG3.

  @Preconditions
    The RG3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG3 low (0)
    IO_RG3_SetLow();
    </code>

 */
#define IO_RG3_SetLow()           _LATG3 = 0
/**
  @Summary
    Toggles the GPIO pin, RG3, using LATG3.

  @Description
    Toggles the GPIO pin, RG3, using LATG3.

  @Preconditions
    The RG3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG3
    IO_RG3_SetToggle();
    </code>

 */
#define IO_RG3_Toggle()           _LATG3 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG3.

  @Description
    Reads the value of the GPIO pin, RG3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG3
    postValue = IO_RG3_GetValue();
    </code>

 */
#define IO_RG3_GetValue()         _RG3
/**
  @Summary
    Configures the GPIO pin, RG3, as an input.

  @Description
    Configures the GPIO pin, RG3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG3 as an input
    IO_RG3_SetDigitalInput();
    </code>

 */
#define IO_RG3_SetDigitalInput()  _TRISG3 = 1
/**
  @Summary
    Configures the GPIO pin, RG3, as an output.

  @Description
    Configures the GPIO pin, RG3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG3 as an output
    IO_RG3_SetDigitalOutput();
    </code>

 */
#define IO_RG3_SetDigitalOutput() _TRISG3 = 0

/**
    Section: Function Prototypes
 */
/**
  @Summary
    Configures the pin settings of the PIC24FJ128GA310
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the MPLAB® Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

 */
void PIN_MANAGER_Initialize(void);

#endif
