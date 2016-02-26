/**
  @Generated MPLAB® Code Configurator Source File

  @Company:
    Microchip Technology Inc.

  @File Name:
    mcc.c

  @Summary:
    This is the mcc.c file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC24FJ128GA310
        Driver Version    :  1.02
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

// Configuration bits: selected in the GUI

// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F    // Deep Sleep Watchdog Timer Postscale Select bits->1:68719476736 (25.7 Days)
#pragma config DSSWEN = ON    // DSEN Bit Enable->Deep Sleep is controlled by the register bit DSEN
#pragma config DSWDTOSC = LPRC    // DSWDT Reference Clock Select->DSWDT uses LPRC as reference clock
#pragma config DSBOREN = ON    // Deep Sleep BOR Enable bit->DSBOR Enabled
#pragma config DSWDTEN = ON    // Deep Sleep Watchdog Timer Enable->DSWDT Enabled

// CONFIG3
#pragma config WPDIS = WPDIS    // Segment Write Protection Disable->Disabled
#pragma config WPFP = WPFP127    // Write Protection Flash Page Segment Boundary->Page 127 (0x1FC00)
#pragma config WPEND = WPENDMEM    // Segment Write Protection End Page Select->Write Protect from WPFP to the last page of memory
#pragma config VBTBOR = ON    // VBAT BOR enable bit->VBAT BOR enabled
#pragma config BOREN = ON    // Brown-out Reset Enable->Brown-out Reset Enable
#pragma config WPCFG = WPCFGDIS    // Write Protect Configuration Page Select->Disabled
#pragma config SOSCSEL = ON    // SOSC Selection bits->SOSC circuit selected
#pragma config WDTWIN = PS25_0    // Watch Dog Timer Window Width->Watch Dog Timer Window Width is 25 percent

// CONFIG2
#pragma config POSCMD = NONE    // Primary Oscillator Select->Primary Oscillator Disabled
#pragma config IESO = ON    // Internal External Switchover->Enabled
#pragma config IOL1WAY = ON    // IOLOCK One-Way Set Enable bit->Once set, the IOLOCK bit cannot be cleared
#pragma config FNOSC = FRCPLL    // Initial Oscillator Select->Fast RC Oscillator with PLL module (FRCPLL)
#pragma config BOREN1 = EN    // BOR Override bit->BOR Enabled [When BOREN=1]
#pragma config ALTVREF = DLT_AV_DLT_CV    // Alternate VREF/CVREF Pins Selection bit->Voltage reference input, ADC =RA9/RA10 Comparator =RA9,RA10
#pragma config FCKSM = CSECME    // Clock Switching and Fail-Safe Clock Monitor Configuration bits->Clock switching is enabled, Fail-Safe Clock Monitor is enabled
#pragma config OSCIOFCN = OFF    // OSCO Pin Configuration->OSCO/CLKO/RC15 functions as CLKO (FOSC/2)

// CONFIG1
#pragma config WDTPS = PS32768    // Watchdog Timer Postscaler Select->1:32,768
#pragma config LPCFG = OFF    // Low power regulator control->Disabled
#pragma config ICS = PGx1    // Emulator Pin Placement Select bits->Emulator functions are shared with PGEC1/PGED1
#pragma config FWPSA = PR128    // WDT Prescaler Ratio Select->1:128
#pragma config WINDIS = OFF    // Windowed WDT Disable->Standard Watchdog Timer
#pragma config GWRP = OFF    // General Segment Write Protect->Disabled
#pragma config GCP = OFF    // General Segment Code Protect->Code protection is disabled
#pragma config FWDTEN = WDT_DIS    // Watchdog Timer Enable->WDT disabled in hardware; SWDTEN bit disabled
#pragma config JTAGEN = OFF    // JTAG Port Enable->Disabled

#include "mcc.h"

void SYSTEM_Initialize(void) {
    OSCILLATOR_Initialize();
    PIN_MANAGER_Initialize();
    INTERRUPT_Initialize();
    SPI1_Initialize();
    OC1_Initialize();
    OC2_Initialize();
    OC3_Initialize();
    OC4_Initialize();
    OC5_Initialize();
    TMR2_Initialize();
    TMR3_Initialize();
    UART1_Initialize();
    UART2_Initialize();
    ADC1_Initialize();
    IC1_Initialize();
    TMR4_Initialize();
    EXT_INT_Initialize();
}

void OSCILLATOR_Initialize(void) {
    // DOZEN disabled; DOZE 1:8; RCDIV FRC/2; ROI disabled; 
    CLKDIV = 0x3100;
    // Set the secondary oscillator

}

/**
 End of File
 */