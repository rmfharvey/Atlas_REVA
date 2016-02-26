/**
  OC4 Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    oc4.h

  @Summary
    This is the generated header file for the OC4 driver using MPLAB® Code Configurator

  @Description
    This header file provides APIs for driver for OC4.
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

#ifndef _OC4_H
#define _OC4_H

/**
  Section: Included Files
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

    /**
      Section: Data Types
     */

    /** OC4 Fault Number

      @Summary
        Defines the fault number

      @Description
        This type defines the various faults supported.

      Remarks:
        None
     */

    typedef enum {
        /* Fault 0 */
        OC4_FAULT0 /*DOM-IGNORE-BEGIN*/ = 0, /*DOM-IGNORE-END*/
        /* Fault 1 */
        OC4_FAULT1 /*DOM-IGNORE-BEGIN*/ = 1, /*DOM-IGNORE-END*/
        /* Fault 2 */
        OC4_FAULT2 /*DOM-IGNORE-BEGIN*/ = 2/*DOM-IGNORE-END*/
    } OC4_FAULTS;

    /**
    Deprecated Fault enumeration
     */
    typedef enum {
        /* Fault 0 */
        DRV_OC4_FAULT0 /*DOM-IGNORE-BEGIN*/ = 0, /*DOM-IGNORE-END*/
        /* Fault 1 */
        DRV_OC4_FAULT1 /*DOM-IGNORE-BEGIN*/ = 1, /*DOM-IGNORE-END*/
        /* Fault 2 */
        DRV_OC4_FAULT2 /*DOM-IGNORE-BEGIN*/ = 2/*DOM-IGNORE-END*/
    } DRV_OC4_FAULTS;

    /**
      Section: Interface Routines
     */


    /**
      @Summary
        This function initializes OC instance : 4

      @Description
        This routine initializes the OC4 driver instance for : 4
        index, making it ready for clients to open and use it.
        This routine must be called before any other OC4 routine is called.
	
      @Preconditions
        None.

      @Param
        None.

      @Returns
        None.

      @Example
        <code>
        uint16_t priVal,secVal;
        bool completeCycle = false;
        priVal = 0x1000;
        secVal = 0x2000;
        OC4_FAULTS faultNum = OC4_FAULT0;

        OC4_Initialize();
    
        OC4_CentreAlignedPWMConfig( priVal, secVal );
  
        OC4_Start();

        while(1)
        {
            faultStat =  OC4_FaultStatusGet( faultNum );

            if(faultStat)
            {
                OC4_FaultStatusClear( faultNum );
            }

            completeCycle = OC4_IsCompareCycleComplete( void );
            if(completeCycle)
            {
                OC4_Stop();
            }
        }
        </code>

     */

    void OC4_Initialize(void);
    /**
        void DRV_OC4_Initialize(void)
     */
    void DRV_OC4_Initialize(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_Initialize instead. ")));

    /**
      @Summary
        Maintains the driver's state machine and implements its ISR

      @Description
        This routine is used to maintain the driver's internal state
        machine and implement its ISR for interrupt-driven implementations.
  
      @Preconditions
        None.

      @Param
        None.

      @Returns
        None.

      @Example
        <code>
        while (true)
        {
            OC4_Tasks();

            // Do other tasks
        }
        </code>
    
     */
    void OC4_Tasks(void);

    /**
        void DRV_OC4_Tasks(void)
     */
    void DRV_OC4_Tasks(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_Tasks instead. ")));


    /**
      @Summary
        Enables the OC module with the corresponding operation mode.

      @Description
        This routine enables the OC module with the corresponding operation mode.

      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        None.

      @Returns
        None.

      @Example 
        Refer to OC4_Initialize() for an example	
 
     */
    void OC4_Start(void);

    /**
        void DRV_OC4_Start(void)
     */
    void DRV_OC4_Start(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_Start instead. ")));

    /**
      @Summary
        Disables the OC module.

      @Description
        This routine disables the OC module.

      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        None.

      @Returns
        None.

     */
    void OC4_Stop(void);

    /**
        void DRV_OC4_Stop(void)
     */
    void DRV_OC4_Stop(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_Stop instead. ")));

    /**
      @Summary
        Sets the primary compare value.

      @Description
        This routine sets the primary compare value.

      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        None.

      @Returns
        None.

      @Example 
        <code>
            uint16_t value = 0x1000;
            OC4_SingleCompareValueSet( value );
        <code>
     */

    void OC4_SingleCompareValueSet(uint16_t value);

    /**
        void DRV_OC4_SingleCompareValueSet(uint16_t value)
     */
    void DRV_OC4_SingleCompareValueSet(uint16_t value) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_SingleCompareValueSet instead. ")));

    /**
      @Summary
        Sets the primary and secondary compare value.

      @Description
        This routine sets the primary and secondary compare value.

      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        priVal - 16 bit primary compare value.
        secVal - 16 bit primary compare value.	

      @Returns
        None.

      @Example 
        <code>
            uint16_t priVal = 0x1000;
            uint16_t secVal = 0x2000;
            OC4_DualCompareValueSet( priVal,secVal );
        <code>
  	
     */

    void OC4_DualCompareValueSet(uint16_t priVal, uint16_t secVal);

    /**
        void DRV_OC4_DualCompareValueSet(uint16_t priVal, uint16_t secVal)
     */
    void DRV_OC4_DualCompareValueSet(uint16_t priVal, uint16_t secVal) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_DualCompareValueSet instead. ")));

    /**
      @Summary
        Sets the primary and secondary compare value for Center Aligned PWM

      @Description
        This routine sets the primary and secondary compare value for Center Aligned PWM.

      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        priVal - 16 bit primary compare value.
        secVal - 16 bit primary compare value.	
	
      @Returns
        None.

      @Example 
        Refer to OC4_Initialize() for an example

     */
    void OC4_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal);

    /**
        void DRV_OC4_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal)
     */
    void DRV_OC4_CentreAlignedPWMConfig(uint16_t priVal, uint16_t secVal) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_CentreAlignedPWMConfig instead. ")));

    /**
      @Summary
        Sets the primary and secondary compare value for Edge Aligned PWM.

      @Description
        This routine sets the primary and secondary compare value for Edge Aligned PWM.

      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        priVal - 16 bit primary compare value.
        secVal - 16 bit primary compare value.	

      @Returns
        None.

      @Example 
        <code>
            uint16_t priVal = 0x1000;
            uint16_t secVal = 0x2000;
            OC4_EdgeAlignedPWMConfig( priVal,secVal );
        <code> 
 	
     */
    void OC4_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal);

    /**
        void DRV_OC4_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal)
     */
    void DRV_OC4_EdgeAlignedPWMConfig(uint16_t priVal, uint16_t secVal) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_EdgeAlignedPWMConfig instead. ")));

    /**
      @Summary
        Gets the status of the compare cycle completion.

      @Description
        This routine gets the status of the compare cycle completion.

      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        None.

      @Returns
        Boolean value describing the current status of the cycle completion. Returns
        true  : When the compare cycle has completed. 
        false : When the compare cycle has not completed. 

      @Example 
        Refer to OC4_Initialize() for an example
	

     */
    bool OC4_IsCompareCycleComplete(void);

    /**
        bool DRV_OC4_IsCompareCycleComplete(void)
     */
    bool DRV_OC4_IsCompareCycleComplete(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_IsCompareCycleComplete instead. ")));

    /**
      @Summary
        Gets the status of the PWM fault condition occurrence.

      @Description
        This routine gets the status of the PWM fault condition occurrence.
  
      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        faultNum - The fault number

      @Returns
        boolean value describing the occurrence of the fault condition.
        true  : When the specified fault has occurred.
        false : When the specified fault has not occurred.
	
      @Example 
        Refer to OC4_Initialize() for an example 
 

     */
    bool OC4_FaultStatusGet(OC4_FAULTS faultNum);

    /**
        bool DRV_OC4_FaultStatusGet(DRV_OC4_FAULTS faultNum)
     */
    bool DRV_OC4_FaultStatusGet(DRV_OC4_FAULTS faultNum) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_FaultStatusGet instead. ")));

    /**
      @Summary
        Clears the status of the PWM fault condition occurrence.

      @Description
        This routine clears the status of the PWM fault condition occurrence.
	
      @Preconditions
        None.	

      @Param
        faultNum - The fault number
  
      @Returns
        None.

      @Example 
        Refer to OC4_Initialize() for an example 
  	
     */
    void OC4_FaultStatusClear(OC4_FAULTS faultNum);

    /**
        void DRV_OC4_FaultStatusClear(DRV_OC4_FAULTS faultNum)
     */
    void DRV_OC4_FaultStatusClear(DRV_OC4_FAULTS faultNum) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_FaultStatusClear instead. ")));

    /**
      @Summary
        Sets the manual trigger

      @Description
        This routine sets the manual trigger
	
      @Preconditions
        OC4_Initialize function should have been called 

      @Param
        None
 
      @Returns
        None.
	
      @Example 
        Refer to OC4_TriggerStatusGet() for an example	
 
     */
    void OC4_ManualTriggerSet(void);

    /**
      @Summary
        Gets the status of the timer trigger.

      @Description
        This routine gets the status of the timer trigger source if it has been triggered.
	
      @Preconditions
        OC4_Initialize function should have been called 
	
      @Param
        None
	
      @Returns
        Boolean value describing the timer trigger status.
        true  : When the timer source has triggered and is running 
        false : When the timer has not triggered and being held clear 

      @Example 
        <\code>	
        if(OC4_TriggerStatusGet())
        {
            OC4_TriggerStatusClear();
        }
        <\code>	
     */
    bool OC4_TriggerStatusGet(void);

    /**
        bool DRV_OC4_TriggerStatusGet(void)
     */
    bool DRV_OC4_TriggerStatusGet(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_TriggerStatusGet instead. ")));

    /**
      @Summary
        Clears the status of the timer trigger.

      @Description
        This routine clears the status of the timer trigger.
	
      @Preconditions
        OC4_Initialize function should have been called

      @Param
        None
	
      @Returns
        None.
	
      @Example 
        Refer to OC4_TriggerStatusGet() for an example	

     */
    void OC4_TriggerStatusClear(void);

    /**
        void DRV_OC4_TriggerStatusClear(void)
     */
    void DRV_OC4_TriggerStatusClear(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse OC4_TriggerStatusClear instead. ")));
#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif //_OC4_H

/**
 End of File
 */
