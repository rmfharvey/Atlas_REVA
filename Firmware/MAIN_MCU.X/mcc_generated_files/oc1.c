
/**
  OC1 Generated Driver API Source File

  @Company
    Microchip Technology Inc.

  @File Name
    oc1.c

  @Summary
    This is the generated source file for the OC1 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for driver for OC1.
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
#include "oc1.h"

/** OC Mode.

  @Summary
    Defines the OC Mode.

  @Description
    This data type defines the OC Mode of operation.

 */

static uint16_t gOC1Mode;

/**
  Section: Driver Interface
 */


void OC1_Initialize(void) {
    // OCFLT0 disabled; OCTSEL TMR2; OCFLT1 disabled; OCM Edge-Aligned PWM mode; OCSIDL disabled; ENFLT2 disabled; ENFLT1 disabled; TRIGMODE Only Software; OCFLT2 disabled; ENFLT0 disabled; 
    OC1CON1 = 0x0006;
    // OCTRIS disabled; OCINV disabled; FLTMD Cycle; SYNCSEL Self; FLTOUT disabled; TRIGSTAT disabled; DCB Start of instruction cycle; FLTTRIEN disabled; OC32 disabled; OCTRIG disabled; 
    OC1CON2 = 0x001F;
    // OC1RS 399; 
    OC1RS = 0x018F;
    // OC1R 360; 
    OC1R = 0x0168;
    // OC1TMR 0; 
    OC1TMR = 0x0000;

    gOC1Mode = OC1CON1bits.OCM;
}

/**
    void DRV_OC1_Initialize (void)
 */
void DRV_OC1_Initialize(void) {
    OC1_Initialize();
}

void OC1_Tasks(void) {
    if (IFS0bits.OC1IF) {
        IFS0bits.OC1IF = 0;
    }
}

/**
    void DRV_OC1_Tasks (void)
 */
void DRV_OC1_Tasks(void) {
    OC1_Tasks();
}

void OC1_Start(void) {
    OC1CON1bits.OCM = gOC1Mode;
}

/**
    void DRV_OC1_Start (void)
 */
void DRV_OC1_Start(void) {
    OC1_Start();
}

void OC1_Stop(void) {
    OC1CON1bits.OCM = 0;
}

/**
    void DRV_OC1_Stop (void)
 */
void DRV_OC1_Stop(void) {
    OC1_Stop();
}

void OC1_SingleCompareValueSet(uint16_t value) {
    OC1R = value;
}

/**
    void DRV_OC1_SingleCompareValueSet (uint16_t value)
 */
void DRV_OC1_SingleCompareValueSet(uint16_t value) {
    OC1_SingleCompareValueSet(value);
}

void OC1_DualCompareValueSet(uint16_t priVal, uint16_t secVal) {
    OC1R = priVal;

    OC1RS = secVal;
}

/**
    void DRV_OC1_DualCompareValueSet (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC1_DualCompareValueSet(uint16_t priVal, uint16_t secVal) {
    OC1_DualCompareValueSet(priVal, secVal);
}

void OC1_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC1R = priVal;

    OC1RS = secVal;
}

/**
    void DRV_OC1_CentreAlignedPWMConfig (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC1_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC1_CentreAlignedPWMConfig(priVal, secVal);
}

void OC1_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC1R = priVal;

    OC1RS = secVal;
}

/**
    void DRV_OC1_EdgeAlignedPWMConfig (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC1_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC1_EdgeAlignedPWMConfig(priVal, secVal);
}

bool OC1_IsCompareCycleComplete(void) {
    return (IFS0bits.OC1IF);
}

/**
    bool DRV_OC1_IsCompareCycleComplete (void)
 */
bool DRV_OC1_IsCompareCycleComplete(void) {
    return (OC1_IsCompareCycleComplete());
}

bool OC1_FaultStatusGet(OC1_FAULTS faultNum) {
    bool status;
    /* Return the status of the fault condition */

    switch (faultNum) {
        case OC1_FAULT0:status = OC1CON1bits.OCFLT0;
            break;
        case OC1_FAULT1:status = OC1CON1bits.OCFLT1;
            break;

        case OC1_FAULT2:status = OC1CON1bits.OCFLT2;
            break;
        default:
            break;

    }
    return (status);
}

/**
    bool DRV_OC1_FaultStatusGet (DRV_OC1_FAULTS faultNum)
 */
bool DRV_OC1_FaultStatusGet(DRV_OC1_FAULTS faultNum) {
    return (OC1_FaultStatusGet(faultNum));
}

void OC1_FaultStatusClear(OC1_FAULTS faultNum) {

    switch (faultNum) {
        case OC1_FAULT0:OC1CON1bits.OCFLT0 = 0;
            break;
        case OC1_FAULT1:OC1CON1bits.OCFLT1 = 0;
            break;

        case OC1_FAULT2:OC1CON1bits.OCFLT2 = 0;
            break;
        default:
            break;
    }
}

/**
    void DRV_OC1_FaultStatusClear (DRV_OC1_FAULTS faultNum)
 */
void DRV_OC1_FaultStatusClear(DRV_OC1_FAULTS faultNum) {
    OC1_FaultStatusClear(faultNum);
}

void OC1_ManualTriggerSet(void) {
    OC1CON2bits.TRIGSTAT = true;
}

bool OC1_TriggerStatusGet(void) {
    return ( OC1CON2bits.TRIGSTAT);
}

/**
    bool DRV_OC1_TriggerStatusGet (void)
 */
bool DRV_OC1_TriggerStatusGet(void) {
    return (OC1_TriggerStatusGet());
}

void OC1_TriggerStatusClear(void) {
    /* Clears the trigger status */
    OC1CON2bits.TRIGSTAT = 0;
}

/**
    void DRV_OC1_TriggerStatusClear (void)
 */
void DRV_OC1_TriggerStatusClear(void) {
    OC1_TriggerStatusClear();
}
/**
 End of File
 */
