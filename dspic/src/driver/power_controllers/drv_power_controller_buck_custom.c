//=======================================================================================================
// Copyright(c) 2018 Microchip Technology Inc. and its subsidiaries.
// Subject to your compliance with these terms, you may use Microchip software and any derivatives
// exclusively with Microchip products. It is your responsibility to comply with third party license
// terms applicable to your use of third-party software (including open source software) that may
// accompany Microchip software.
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
// APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND
// FITNESS FOR A PARTICULAR PURPOSE.
// IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
// LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
// MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT
// ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT
// EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//=======================================================================================================

//=======================================================================================================
// @file drv_power_controller_buck_custom.c
//
// @brief power controller functions for buck converter
//
// @author M91406
// @author M52409
// @author M91281
//
// @date July 9, 2019, 1:10 PM
//=======================================================================================================

#include <stdbool.h>
#include <stdint.h>
#include <xc.h>         // include processor specific defines and functions

#include "driver/power_controllers/drv_power_controller_buck_generic.h"
#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/npnz16b.h"
#include "driver/power_controllers/c2p2z_buck.h"
#include "init_pwm.h"
#include "init_acmp.h"
#include "init_adc.h"

//=======================================================================================================
// local defines
//=======================================================================================================
// Buck instance 1 specific defines:
#define BUCK1_ADC_REFERENCE     3.3             // 3.3 Volts ==> maximum ADC-Value
#define BUCK1_ADC_RESOLUTION    4095UL          // 12 bits
#define BUCK1_FEEDBACK_BGAIN    0.5             // 1k /(1k+1k)

POWER_CONTROLLER_DATA_t pwrCtrlBuck1_Data;      // data instance for the buck converter


//=======================================================================================================
// @brief   returns the Output Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllerBuck1_GetOutputVoltage()
{
    return (double)(((unsigned long)pwrCtrlBuck1_Data.voltageOutput * BUCK1_ADC_REFERENCE) / (BUCK1_FEEDBACK_BGAIN * BUCK1_ADC_RESOLUTION));
}


//=======================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//=======================================================================================================
void Drv_PowerControllerBuck1_SetOutputVoltageReference(POWER_CONTROLLER_DATA_t* pPCData, double newVoltRef)
{
    pPCData->voltageRef_softStart = ((newVoltRef * BUCK1_ADC_RESOLUTION * BUCK1_FEEDBACK_BGAIN) / BUCK1_ADC_REFERENCE);
}


//=======================================================================================================
// @brief   sets the Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage
//=======================================================================================================
void Drv_PowerControllerBuck1_SetOutputVoltageReference_mV(POWER_CONTROLLER_DATA_t* pPCData, uint32_t newVoltRef_mV)
{
    pPCData->voltageRef_softStart = ((((uint32_t)newVoltRef_mV * BUCK1_ADC_RESOLUTION) * BUCK1_FEEDBACK_BGAIN) / BUCK1_ADC_REFERENCE) / 1000;
}

void Drv_PowerControllerBuck1_EnableControlLoop(void)
{
    c2p2z_buck.status.flag.enable = 1;  // Start the control loop for buck
    PG1IOCONLbits.OVRENH = 0;           // User override disabled for PWMxH Pin
    PG1IOCONLbits.OVRENL = 0;           // User override disabled for PWMxL Pin
}

void Drv_PowerControllerBuck1_DisableControlLoop(void)
{
    c2p2z_buck.status.flag.enable = 0;  // Stop the control loop for buck
    PG1IOCONLbits.OVRENH = 1;           // User override disabled for PWMxH Pin
    PG1IOCONLbits.OVRENL = 1;           // User override disabled for PWMxL Pin
    //TODO: should we set some output pin for safety reasons???
}

//=======================================================================================================
// @brief   Initializes the Buck Power Converter - Instance 1
// @note    In this routine all the application specific custom functions are implemented
//=======================================================================================================
void Drv_PowerControllerBuck1_Init(bool autostart)
{
    Drv_PowerControllerBuck_Init(&pwrCtrlBuck1_Data, autostart);

/*
    pPCData->voltageRef_compensator = 0;                // 2047 for 3.3V
    pPCData->voltageOutput = 0;
    pPCData->flags.value = 0;                           //reset everything
    pPCData->flags.bits.adc_active = false;
    // Reset state machine to the beginning
    buckPC_GotoState(pPCData, PCS_STARTUP_PERIPHERALS);
//    Drv_PowerControllerBuck_SetOutputVoltageReference_mV(pPCData, outputVoltageRef_mV);
    Drv_PowerControllerBuck_CalculateVoltageLimits(pPCData);
  */
            
    pwrCtrlBuck1_Data.voltageRef_rampStep = 4;              // 4 adc values per 100µs
    pwrCtrlBuck1_Data.powerInputOk_waitTime_100us = 1000;   // 100ms input power stabilization delay
    pwrCtrlBuck1_Data.powerOutputOk_waitTime_100us = 1000;  // 100ms output power stabilization delay
    pwrCtrlBuck1_Data.ftkEnableControlLoop = Drv_PowerControllerBuck1_EnableControlLoop;
    pwrCtrlBuck1_Data.ftkDisableControlLoop = Drv_PowerControllerBuck1_DisableControlLoop;
    
    init_buck_trig_pwm();   // Set up auxiliary PWM for buck converter
    init_buck_pwm();        // Set up buck converter PWM
    init_buck_acmp();       // Set up buck converter peak current comparator/DAC
    init_buck_adc();        // Set up buck converter ADC (voltage feedback only)
   
    c2p2z_buck_Init();
    c2p2z_buck.ADCTriggerOffset = VOUT_ADC_TRIGGER_DELAY;
    c2p2z_buck.ptrADCTriggerRegister = &PG3TRIGA;
    c2p2z_buck.InputOffset = 0;
    c2p2z_buck.ptrControlReference = &(pwrCtrlBuck1_Data.voltageRef_compensator);    //TODO: wrong pointer!
    c2p2z_buck.ptrSource = &ADCBUF13;
    c2p2z_buck.ptrTarget = &DAC1DATH;
    c2p2z_buck.MaxOutput = 3600;
    c2p2z_buck.MinOutput = 10;
    c2p2z_buck.status.flag.enable = 0;
}


//=======================================================================================================
// @brief   Interrupt routine for calling the buck c2p2z compensator and sampling the Output Voltage
//=======================================================================================================
void __attribute__((__interrupt__, auto_psv)) _ADCAN13Interrupt(void)
{
    c2p2z_buck_Update(&c2p2z_buck);     //call the compensator as soon as possible
    // the readout of the ADC register is mandatory to make the reset of the interrupt flag stick
    // if we would not read from the ADC register then the interrupt flag would be set immediately after resetting it
    pwrCtrlBuck1_Data.voltageOutput =  ADCBUF13;
    pwrCtrlBuck1_Data.flags.bits.adc_active = true;
    _ADCAN13IF = 0;  // Clear the ADCANx interrupt flag. read from ADCBUFx first to make it stick
    
    //TODO: discuss, if we should call the buck_Update routine at first or after resetting the interrupt flag?
}

