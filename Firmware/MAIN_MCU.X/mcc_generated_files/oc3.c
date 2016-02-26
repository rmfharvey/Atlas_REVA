
/**
  OC3 Generated Driver API Source File

  @Company
    Microchip Technology Inc.

  @File Name
    oc3.c

  @Summary
    This is the generated source file for the OC3 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for driver for OC3.
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
#include "oc3.h"

/** OC Mode.

  @Summary
    Defines the OC Mode.

  @Description
    This data type defines the OC Mode of operation.

 */

static uint16_t gOC3Mode;

/**
  Section: Driver Interface
 */


void OC3_Initialize(void) {
    // OCFLT0 disabled; OCTSEL TMR2; OCFLT1 disabled; OCM Edge-Aligned PWM mode; OCSIDL disabled; ENFLT2 disabled; ENFLT1 disabled; TRIGMODE Only Software; OCFLT2 disabled; ENFLT0 disabled; 
    OC3CON1 = 0x0006;
    // OCTRIS disabled; OCINV disabled; FLTMD Cycle; SYNCSEL Self; FLTOUT disabled; TRIGSTAT disabled; DCB Start of instruction cycle; FLTTRIEN disabled; OC32 disabled; OCTRIG disabled; 
    OC3CON2 = 0x001F;
    // OC3RS 399; 
    OC3RS = 0x018F;
    // OC3R 360; 
    OC3R = 0x0168;
    // OC3TMR 0; 
    OC3TMR = 0x0000;

    gOC3Mode = OC3CON1bits.OCM;
}

/**
    void DRV_OC3_Initialize (void)
 */
void DRV_OC3_Initialize(void) {
    OC3_Initialize();
}

void OC3_Tasks(void) {
    if (IFS1bits.OC3IF) {
        IFS1bits.OC3IF = 0;
    }
}

/**
    void DRV_OC3_Tasks (void)
 */
void DRV_OC3_Tasks(void) {
    OC3_Tasks();
}

void OC3_Start(void) {
    OC3CON1bits.OCM = gOC3Mode;
}

/**
    void DRV_OC3_Start (void)
 */
void DRV_OC3_Start(void) {
    OC3_Start();
}

void OC3_Stop(void) {
    OC3CON1bits.OCM = 0;
}

/**
    void DRV_OC3_Stop (void)
 */
void DRV_OC3_Stop(void) {
    OC3_Stop();
}

void OC3_SingleCompareValueSet(uint16_t value) {
    OC3R = value;
}

/**
    void DRV_OC3_SingleCompareValueSet (uint16_t value)
 */
void DRV_OC3_SingleCompareValueSet(uint16_t value) {
    OC3_SingleCompareValueSet(value);
}

void OC3_DualCompareValueSet(uint16_t priVal, uint16_t secVal) {
    OC3R = priVal;

    OC3RS = secVal;
}

/**
    void DRV_OC3_DualCompareValueSet (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC3_DualCompareValueSet(uint16_t priVal, uint16_t secVal) {
    OC3_DualCompareValueSet(priVal, secVal);
}

void OC3_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC3R = priVal;

    OC3RS = secVal;
}

/**
    void DRV_OC3_CentreAlignedPWMConfig (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC3_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC3_CentreAlignedPWMConfig(priVal, secVal);
}

void OC3_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC3R = priVal;

    OC3RS = secVal;
}

/**
    void DRV_OC3_EdgeAlignedPWMConfig (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC3_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC3_EdgeAlignedPWMConfig(priVal, secVal);
}

bool OC3_IsCompareCycleComplete(void) {
    return (IFS1bits.OC3IF);
}

/**
    bool DRV_OC3_IsCompareCycleComplete (void)
 */
bool DRV_OC3_IsCompareCycleComplete(void) {
    return (OC3_IsCompareCycleComplete());
}

bool OC3_FaultStatusGet(OC3_FAULTS faultNum) {
    bool status;
    /* Return the status of the fault condition */

    switch (faultNum) {
        case OC3_FAULT0:status = OC3CON1bits.OCFLT0;
            break;
        case OC3_FAULT1:status = OC3CON1bits.OCFLT1;
            break;

        case OC3_FAULT2:status = OC3CON1bits.OCFLT2;
            break;
        default:
            break;

    }
    return (status);
}

/**
    bool DRV_OC3_FaultStatusGet (DRV_OC3_FAULTS faultNum)
 */
bool DRV_OC3_FaultStatusGet(DRV_OC3_FAULTS faultNum) {
    return (OC3_FaultStatusGet(faultNum));
}

void OC3_FaultStatusClear(OC3_FAULTS faultNum) {

    switch (faultNum) {
        case OC3_FAULT0:OC3CON1bits.OCFLT0 = 0;
            break;
        case OC3_FAULT1:OC3CON1bits.OCFLT1 = 0;
            break;

        case OC3_FAULT2:OC3CON1bits.OCFLT2 = 0;
            break;
        default:
            break;
    }
}

/**
    void DRV_OC3_FaultStatusClear (DRV_OC3_FAULTS faultNum)
 */
void DRV_OC3_FaultStatusClear(DRV_OC3_FAULTS faultNum) {
    OC3_FaultStatusClear(faultNum);
}

void OC3_ManualTriggerSet(void) {
    OC3CON2bits.TRIGSTAT = true;
}

bool OC3_TriggerStatusGet(void) {
    return ( OC3CON2bits.TRIGSTAT);
}

/**
    bool DRV_OC3_TriggerStatusGet (void)
 */
bool DRV_OC3_TriggerStatusGet(void) {
    return (OC3_TriggerStatusGet());
}

void OC3_TriggerStatusClear(void) {
    /* Clears the trigger status */
    OC3CON2bits.TRIGSTAT = 0;
}

/**
    void DRV_OC3_TriggerStatusClear (void)
 */
void DRV_OC3_TriggerStatusClear(void) {
    OC3_TriggerStatusClear();
}
/**
 End of File
 */
