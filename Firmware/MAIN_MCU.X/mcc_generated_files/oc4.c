
/**
  OC4 Generated Driver API Source File

  @Company
    Microchip Technology Inc.

  @File Name
    oc4.c

  @Summary
    This is the generated source file for the OC4 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for driver for OC4.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC24FJ128GA310
        Driver Version    :  0.5
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB 	          :  MPLAB X v2.35 or v3.00
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
  Section: Included Files
 */
#include <xc.h>
#include "oc4.h"

/** OC Mode.

  @Summary
    Defines the OC Mode.

  @Description
    This data type defines the OC Mode of operation.

 */

static uint16_t gOC4Mode;

/**
  Section: Driver Interface
 */


void OC4_Initialize(void) {
    // OCFLT0 disabled; OCTSEL TMR2; OCFLT1 disabled; OCM Edge-Aligned PWM mode; OCSIDL disabled; ENFLT2 disabled; ENFLT1 disabled; TRIGMODE Only Software; OCFLT2 disabled; ENFLT0 disabled; 
    OC4CON1 = 0x0006;
    // OCTRIS disabled; OCINV enabled; FLTMD Cycle; SYNCSEL Self; FLTOUT disabled; TRIGSTAT disabled; DCB Start of instruction cycle; FLTTRIEN disabled; OC32 disabled; OCTRIG disabled; 
    OC4CON2 = 0x101F;
    // OC4RS 399; 
    OC4RS = 0x018F;
    // OC4R 360; 
    OC4R = 0x0168;
    // OC4TMR 0; 
    OC4TMR = 0x0000;

    gOC4Mode = OC4CON1bits.OCM;
}

/**
    void DRV_OC4_Initialize (void)
 */
void DRV_OC4_Initialize(void) {
    OC4_Initialize();
}

void OC4_Tasks(void) {
    if (IFS1bits.OC4IF) {
        IFS1bits.OC4IF = 0;
    }
}

/**
    void DRV_OC4_Tasks (void)
 */
void DRV_OC4_Tasks(void) {
    OC4_Tasks();
}

void OC4_Start(void) {
    OC4CON1bits.OCM = gOC4Mode;
}

/**
    void DRV_OC4_Start (void)
 */
void DRV_OC4_Start(void) {
    OC4_Start();
}

void OC4_Stop(void) {
    OC4CON1bits.OCM = 0;
}

/**
    void DRV_OC4_Stop (void)
 */
void DRV_OC4_Stop(void) {
    OC4_Stop();
}

void OC4_SingleCompareValueSet(uint16_t value) {
    OC4R = value;
}

/**
    void DRV_OC4_SingleCompareValueSet (uint16_t value)
 */
void DRV_OC4_SingleCompareValueSet(uint16_t value) {
    OC4_SingleCompareValueSet(value);
}

void OC4_DualCompareValueSet(uint16_t priVal, uint16_t secVal) {
    OC4R = priVal;

    OC4RS = secVal;
}

/**
    void DRV_OC4_DualCompareValueSet (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC4_DualCompareValueSet(uint16_t priVal, uint16_t secVal) {
    OC4_DualCompareValueSet(priVal, secVal);
}

void OC4_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC4R = priVal;

    OC4RS = secVal;
}

/**
    void DRV_OC4_CentreAlignedPWMConfig (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC4_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC4_CentreAlignedPWMConfig(priVal, secVal);
}

void OC4_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC4R = priVal;

    OC4RS = secVal;
}

/**
    void DRV_OC4_EdgeAlignedPWMConfig (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC4_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC4_EdgeAlignedPWMConfig(priVal, secVal);
}

bool OC4_IsCompareCycleComplete(void) {
    return (IFS1bits.OC4IF);
}

/**
    bool DRV_OC4_IsCompareCycleComplete (void)
 */
bool DRV_OC4_IsCompareCycleComplete(void) {
    return (OC4_IsCompareCycleComplete());
}

bool OC4_FaultStatusGet(OC4_FAULTS faultNum) {
    bool status;
    /* Return the status of the fault condition */

    switch (faultNum) {
        case OC4_FAULT0:status = OC4CON1bits.OCFLT0;
            break;
        case OC4_FAULT1:status = OC4CON1bits.OCFLT1;
            break;

        case OC4_FAULT2:status = OC4CON1bits.OCFLT2;
            break;
        default:
            break;

    }
    return (status);
}

/**
    bool DRV_OC4_FaultStatusGet (DRV_OC4_FAULTS faultNum)
 */
bool DRV_OC4_FaultStatusGet(DRV_OC4_FAULTS faultNum) {
    return (OC4_FaultStatusGet(faultNum));
}

void OC4_FaultStatusClear(OC4_FAULTS faultNum) {

    switch (faultNum) {
        case OC4_FAULT0:OC4CON1bits.OCFLT0 = 0;
            break;
        case OC4_FAULT1:OC4CON1bits.OCFLT1 = 0;
            break;

        case OC4_FAULT2:OC4CON1bits.OCFLT2 = 0;
            break;
        default:
            break;
    }
}

/**
    void DRV_OC4_FaultStatusClear (DRV_OC4_FAULTS faultNum)
 */
void DRV_OC4_FaultStatusClear(DRV_OC4_FAULTS faultNum) {
    OC4_FaultStatusClear(faultNum);
}

void OC4_ManualTriggerSet(void) {
    OC4CON2bits.TRIGSTAT = true;
}

bool OC4_TriggerStatusGet(void) {
    return ( OC4CON2bits.TRIGSTAT);
}

/**
    bool DRV_OC4_TriggerStatusGet (void)
 */
bool DRV_OC4_TriggerStatusGet(void) {
    return (OC4_TriggerStatusGet());
}

void OC4_TriggerStatusClear(void) {
    /* Clears the trigger status */
    OC4CON2bits.TRIGSTAT = 0;
}

/**
    void DRV_OC4_TriggerStatusClear (void)
 */
void DRV_OC4_TriggerStatusClear(void) {
    OC4_TriggerStatusClear();
}
/**
 End of File
 */
