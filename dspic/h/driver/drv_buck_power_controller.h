//======================================================================================================================
// Copyright(c) 2018 Microchip Technology Inc. and its subsidiaries.
// Subject to your compliance with these terms, you may use Microchip software and any derivatives exclusively with
// Microchip products. It is your responsibility to comply with third party license terms applicable to your use of
// third-party software (including open source software) that may accompany Microchip software.
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO
// THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
// PURPOSE.
// IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE,
// COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED
// OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY
// ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE
// PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//======================================================================================================================

//======================================================================================================================
// @file drv_buck_power_controller.c
//
// @brief power controller functions for buck converter
//
// @author M91406
// @author M52409
// @author M91281
//
// @date July 9, 2019, 1:10 PM
//======================================================================================================================

#ifndef _DRV_BUCK_POWER_CONTROLLER_H_
#define	_DRV_BUCK_POWER_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif // __cplusplus

//======================================================================================================================
// adjust these defines to have the right under- and overvoltage borders for monitoring the output voltage
//======================================================================================================================
#define BUCK_OVERVOLTAGE_PERCENT     103
#define BUCK_UNDERVOLTAGE_PERCENT     97

//======================================================================================================================
// these flags can be used to detect some over(/under voltage and to implement some fault handling
// they are set by the power controller after the output voltage is ramped up and stable
//======================================================================================================================
extern volatile bool buckPC_OverVoltageFlag;
extern volatile bool buckPC_UnderVoltageFlag;

//======================================================================================================================
// @brief   Initializes all peripherals and data structures of the buck controller like PWN, ADC, DAC, CMP etc.
// @note    call this during booting up the system before you call anything else or the Power Controller
//======================================================================================================================
extern void     Drv_BuckPowerController_Init(void);

//======================================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//======================================================================================================================
extern void     Drv_BuckPowerController_SetOutputVoltageReference(double newVoltRef);

//======================================================================================================================
// @brief   returns the raw ADC value for the Output Voltage
//======================================================================================================================
extern uint16_t Drv_BuckPowerController_GetOutputVoltageRaw(void);

//======================================================================================================================
// @brief   returns the Output Voltage in Volts as a double
//======================================================================================================================
extern double   Drv_BuckPowerController_GetOutputVoltage(void);

//======================================================================================================================
// @brief   Task that runs all the necessary things to do like soft start and voltage monitoring
// @note    call this every 100us from your main scheduler to ensure the right timing
//======================================================================================================================
extern void     Drv_BuckPowerController_Task_100us(void);

#ifdef	__cplusplus
}
#endif // __cplusplus

#endif	// _DRV_BUCK_POWER_CONTROLLER_H_

