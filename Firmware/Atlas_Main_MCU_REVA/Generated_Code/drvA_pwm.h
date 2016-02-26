/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : drvA_pwm.h
**     Project     : Atlas_Main_MCU_REVA
**     Processor   : MK22FN512VDC12
**     Component   : fsl_ftm
**     Version     : Component 1.2.0, Driver 01.00, CPU db: 3.00.000
**     Repository  : KSDK 1.2.0
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-11-14, 13:36, # CodeGen: 1
**     Contents    :
**         FTM_DRV_Init                    - ftm_status_t FTM_DRV_Init(uint32_t instance,const ftm_user_config_t * info);
**         FTM_DRV_Deinit                  - void FTM_DRV_Deinit(uint32_t instance);
**         FTM_DRV_PwmStop                 - void FTM_DRV_PwmStop(uint32_t instance,ftm_pwm_param_t * param,uint8_t channel);
**         FTM_DRV_PwmStart                - ftm_status_t FTM_DRV_PwmStart(uint32_t instance,ftm_pwm_param_t *...
**         FTM_DRV_QuadDecodeStart         - void FTM_DRV_QuadDecodeStart(uint32_t instance,ftm_phase_params_t *...
**         FTM_DRV_QuadDecodeStop          - void FTM_DRV_QuadDecodeStop(uint32_t instance);
**         FTM_DRV_CounterStart            - void FTM_DRV_CounterStart(uint32_t instance,ftm_counting_mode_t...
**         FTM_DRV_CounterStop             - void FTM_DRV_CounterStop(uint32_t instance);
**         FTM_DRV_CounterRead             - uint32_t FTM_DRV_CounterRead(uint32_t instance);
**         FTM_DRV_SetClock                - void FTM_DRV_SetClock(uint8_t instance,ftm_clock_source_t...
**         FTM_DRV_GetClock                - uint32_t FTM_DRV_GetClock(uint8_t instance);
**         FTM_DRV_SetTimeOverflowIntCmd   - void FTM_DRV_SetTimeOverflowIntCmd(uint32_t instance,bool overflowEnable);
**         FTM_DRV_SetFaultIntCmd          - void FTM_DRV_SetFaultIntCmd(uint32_t instance,bool faultEnable);
**         FTM_DRV_SetupChnInputCapture    - void FTM_DRV_SetupChnInputCapture(uint32_t...
**         FTM_DRV_SetupChnOutputCompare   - void FTM_DRV_SetupChnOutputCompare(uint32_t...
**         FTM_DRV_SetupChnDualEdgeCapture - void FTM_DRV_SetupChnDualEdgeCapture(uint32_t...
**         FTM_DRV_IRQHandler              - void FTM_DRV_IRQHandler(uint32_t instance);
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file drvA_pwm.h
** @version 01.00
*/         
/*!
**  @addtogroup drvA_pwm_module drvA_pwm module documentation
**  @{
*/         
#ifndef __drvA_pwm_H
#define __drvA_pwm_H
/* MODULE drvA_pwm. */

/* Include inherited beans */
#include "clockMan1.h"
#include "Cpu.h"

/*! @brief Device instance number */
#define FSL_DRVA_PWM FTM2_IDX

/*! @brief PWM configuration declaration */
extern ftm_pwm_param_t drvA_pwm_ChnConfig0;
    
/*! @brief Basic configuration declaration */
extern ftm_user_config_t drvA_pwm_InitConfig0;
#endif
/* ifndef __drvA_pwm_H */
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
