
/**
  OC5 Generated Driver API Source File

  @Company
    Microchip Technology Inc.

  @File Name
    oc5.c

  @Summary
    This is the generated source file for the OC5 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for driver for OC5.
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
#include "oc5.h"

/** OC Mode.

  @Summary
    Defines the OC Mode.

  @Description
    This data type defines the OC Mode of operation.

 */

static uint16_t gOC5Mode;

/**
  Section: Driver Interface
 */


void OC5_Initialize(void) {
    // OCFLT0 disabled; OCTSEL TMR3; OCFLT1 disabled; OCM Edge-Aligned PWM mode; OCSIDL disabled; ENFLT2 disabled; ENFLT1 disabled; TRIGMODE Only Software; OCFLT2 disabled; ENFLT0 disabled; 
    OC5CON1 = 0x0406;
    // OCTRIS disabled; OCINV disabled; FLTMD Cycle; SYNCSEL Self; FLTOUT disabled; TRIGSTAT disabled; DCB Start of instruction cycle; FLTTRIEN disabled; OC32 disabled; OCTRIG disabled; 
    OC5CON2 = 0x001F;
    // OC5RS 199; 
    OC5RS = 0x00C7;
    // OC5R 100; 
    OC5R = 0x0064;
    // OC5TMR 0; 
    OC5TMR = 0x0000;

    gOC5Mode = OC5CON1bits.OCM;
}

/**
    void DRV_OC5_Initialize (void)
 */
void DRV_OC5_Initialize(void) {
    OC5_Initialize();
}

void OC5_Tasks(void) {
    if (IFS2bits.OC5IF) {
        IFS2bits.OC5IF = 0;
    }
}

/**
    void DRV_OC5_Tasks (void)
 */
void DRV_OC5_Tasks(void) {
    OC5_Tasks();
}

void OC5_Start(void) {
    OC5CON1bits.OCM = gOC5Mode;
}

/**
    void DRV_OC5_Start (void)
 */
void DRV_OC5_Start(void) {
    OC5_Start();
}

void OC5_Stop(void) {
    OC5CON1bits.OCM = 0;
}

/**
    void DRV_OC5_Stop (void)
 */
void DRV_OC5_Stop(void) {
    OC5_Stop();
}

void OC5_SingleCompareValueSet(uint16_t value) {
    OC5R = value;
}

/**
    void DRV_OC5_SingleCompareValueSet (uint16_t value)
 */
void DRV_OC5_SingleCompareValueSet(uint16_t value) {
    OC5_SingleCompareValueSet(value);
}

void OC5_DualCompareValueSet(uint16_t priVal, uint16_t secVal) {
    OC5R = priVal;

    OC5RS = secVal;
}

/**
    void DRV_OC5_DualCompareValueSet (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC5_DualCompareValueSet(uint16_t priVal, uint16_t secVal) {
    OC5_DualCompareValueSet(priVal, secVal);
}

void OC5_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC5R = priVal;

    OC5RS = secVal;
}

/**
    void DRV_OC5_CentreAlignedPWMConfig (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC5_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC5_CentreAlignedPWMConfig(priVal, secVal);
}

void OC5_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC5R = priVal;

    OC5RS = secVal;
}

/**
    void DRV_OC5_EdgeAlignedPWMConfig (uint16_t priVal, uint16_t secVal)
 */
void DRV_OC5_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) {
    OC5_EdgeAlignedPWMConfig(priVal, secVal);
}

bool OC5_IsCompareCycleComplete(void) {
    return (IFS2bits.OC5IF);
}

/**
    bool DRV_OC5_IsCompareCycleComplete (void)
 */
bool DRV_OC5_IsCompareCycleComplete(void) {
    return (OC5_IsCompareCycleComplete());
}

bool OC5_FaultStatusGet(OC5_FAULTS faultNum) {
    bool status;
    /* Return the status of the fault condition */

    switch (faultNum) {
        case OC5_FAULT0:status = OC5CON1bits.OCFLT0;
            break;
        case OC5_FAULT1:status = OC5CON1bits.OCFLT1;
            break;

        case OC5_FAULT2:status = OC5CON1bits.OCFLT2;
            break;
        default:
            break;

    }
    return (status);
}

/**
    bool DRV_OC5_FaultStatusGet (DRV_OC5_FAULTS faultNum)
 */
bool DRV_OC5_FaultStatusGet(DRV_OC5_FAULTS faultNum) {
    return (OC5_FaultStatusGet(faultNum));
}

void OC5_FaultStatusClear(OC5_FAULTS faultNum) {

    switch (faultNum) {
        case OC5_FAULT0:OC5CON1bits.OCFLT0 = 0;
            break;
        case OC5_FAULT1:OC5CON1bits.OCFLT1 = 0;
            break;

        case OC5_FAULT2:OC5CON1bits.OCFLT2 = 0;
            break;
        default:
            break;
    }
}

/**
    void DRV_OC5_FaultStatusClear (DRV_OC5_FAULTS faultNum)
 */
void DRV_OC5_FaultStatusClear(DRV_OC5_FAULTS faultNum) {
    OC5_FaultStatusClear(faultNum);
}

void OC5_ManualTriggerSet(void) {
    OC5CON2bits.TRIGSTAT = true;
}

bool OC5_TriggerStatusGet(void) {
    return ( OC5CON2bits.TRIGSTAT);
}

/**
    bool DRV_OC5_TriggerStatusGet (void)
 */
bool DRV_OC5_TriggerStatusGet(void) {
    return (OC5_TriggerStatusGet());
}

void OC5_TriggerStatusClear(void) {
    /* Clears the trigger status */
    OC5CON2bits.TRIGSTAT = 0;
}

/**
    void DRV_OC5_TriggerStatusClear (void)
 */
void DRV_OC5_TriggerStatusClear(void) {
    OC5_TriggerStatusClear();
}
/**
 End of File
 */
