/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : sensor_spi.c
**     Project     : Atlas_Main_MCU_REVA
**     Processor   : MK22FN512VDC12
**     Component   : fsl_dspi
**     Version     : Component 1.2.0, Driver 01.00, CPU db: 3.00.000
**     Repository  : KSDK 1.2.0
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-11-14, 13:36, # CodeGen: 1
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
** @file sensor_spi.c
** @version 01.00
*/         
/*!
**  @addtogroup sensor_spi_module sensor_spi module documentation
**  @{
*/         

/* sensor_spi. */

#include "Events.h"
#include "sensor_spi.h"

dspi_master_state_t sensor_spi_MasterState;
uint32_t sensor_spi_calculatedBaudRate = 0;
  
const dspi_device_t sensor_spi_BusConfig0 = {
  .bitsPerSec = 2097152U,
  .dataBusConfig.bitsPerFrame = 8U,
  .dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
  .dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge,
  .dataBusConfig.direction = kDspiMsbFirst,
};

const dspi_master_user_config_t sensor_spi_MasterConfig0 = {
  .whichCtar = kDspiCtar0,
  .isSckContinuous = false,
  .isChipSelectContinuous = false,
  .whichPcs = kDspiPcs0,
  .pcsPolarity = kDspiPcs_ActiveHigh,
};

const dspi_slave_user_config_t sensor_spi_SlaveConfig0 = {
  .dataConfig.bitsPerFrame = 8U,
  .dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
  .dataConfig.clkPhase = kDspiClockPhase_FirstEdge,
  .dataConfig.direction = kDspiMsbFirst,
  .dummyPattern = 0U,
};


/* END sensor_spi. */

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
