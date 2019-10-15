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
// @file drv_power_controller_buck_generic.c
//
// @brief   power controller functions for buck converter
// @note    in this file are only generic functions for the buck converter,
//          every application specific customized code is in the file drv_power_controller_custom_buck.c
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


#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_buck_generic.h"

//=======================================================================================================
// local defines
//=======================================================================================================

//=======================================================================================================
// adjust these defines to have the right under- and overvoltage borders for monitoring the output voltage
//=======================================================================================================
#define BUCK_OVERVOLTAGE_PERCENT     103
#define BUCK_UNDERVOLTAGE_PERCENT     97

volatile static uint16_t vin_avg = 0;      // Averaging buffer for Vin measurement

static inline void Drv_PowerControllerBuck_MonitorVoltageLimits(POWER_CONTROLLER_DATA_t* pPCData);

//=======================================================================================================
// @brief   returns the raw ADC value for the Output Voltage
//=======================================================================================================
uint16_t Drv_PowerControllerBuck_GetOutputVoltageRaw(POWER_CONTROLLER_DATA_t* pPCData)
{
    return pPCData->voltageOutput;
}

//=======================================================================================================
// @brief   internal helping function for the state machine to set the new state
// @note    the also resets the timer automatically
//=======================================================================================================
void buckPC_GotoState(POWER_CONTROLLER_DATA_t* pPCData, PWR_CTRL_STATE_e newState)
{
    pPCData->averageCounter = 0;
    pPCData->timeCounter = 0;
    pPCData->pc_state_internal = newState;
}


//=======================================================================================================
// @brief   Dummy function for FaultDetection
// @note    This function is only for Initialization in the generic part to make the system work
// @note    It should be overwritten in the custom part of the power controller to make use of this
//          function for startup and shutdown of the power controller
//=======================================================================================================
bool Buck_FaultDetectedDummy(void)
{
    return false;
}

#define BUCK_VOLTAGELIMIT_NOISE 2
static inline void Buck_CalculateVoltageLimits(POWER_CONTROLLER_DATA_t* pPCData)
{
    pPCData->UnderVoltageLimit = (uint32_t) (((uint32_t)pPCData->voltageRef_compensator) * BUCK_UNDERVOLTAGE_PERCENT) / 100;
    if (pPCData->UnderVoltageLimit <= BUCK_VOLTAGELIMIT_NOISE)
        pPCData->UnderVoltageLimit = 0;
    else
        pPCData->UnderVoltageLimit -= BUCK_VOLTAGELIMIT_NOISE;

    pPCData->OverVoltageLimit  = (uint32_t) (((uint32_t)pPCData->voltageRef_compensator) * BUCK_OVERVOLTAGE_PERCENT) / 100;
    if (pPCData->OverVoltageLimit < (UINT16_MAX - BUCK_VOLTAGELIMIT_NOISE))
        pPCData->OverVoltageLimit += BUCK_VOLTAGELIMIT_NOISE;
    else
        pPCData->OverVoltageLimit = UINT16_MAX;
}

//=======================================================================================================
// @brief   sets the Internal Output Voltage Reference
// @note    this function is only for internal use
//=======================================================================================================
static inline void Buck_Compensator_SetOutputVoltageRef(POWER_CONTROLLER_DATA_t* pPCData, uint16_t newReference)
{
    pPCData->voltageRef_compensator = newReference;
    Buck_CalculateVoltageLimits(pPCData);
}

//=======================================================================================================
// @brief   Initializes all peripherals and data structures of the buck controller like PWM, ADC, DAC, CMP etc.
// @note    call this during booting up the system before you call anything else or the Power Controller
//=======================================================================================================
void Drv_PowerControllerBuck_Init(POWER_CONTROLLER_DATA_t* pPCData, bool autostart)
{
    Buck_Compensator_SetOutputVoltageRef(pPCData, 0);     // 2047 for 3.3V
    pPCData->voltageRef_softStart = 0;                  // 2047 for 3.3V
    pPCData->voltageOutput = 0;
    pPCData->flags.value = 0;                           // reset everything
    pPCData->flags.bits.adc_active = false;
    pPCData->flags.bits.auto_start = autostart;
    pPCData->ftkFaultDetected = Buck_FaultDetectedDummy;
    buckPC_GotoState(pPCData, PCS_STARTUP_PERIPHERALS); // reset state machine
}


//=======================================================================================================
// @brief   Task that runs all the necessary things to do like soft start and voltage monitoring
// @note    call this every 100us from your main scheduler to ensure the right timing
//=======================================================================================================
void Drv_PowerControllerBuck_Task_100us(POWER_CONTROLLER_DATA_t* pPCData)
{
    //Monitor over- and undervoltage every time before calling the statemachine:
    Drv_PowerControllerBuck_MonitorVoltageLimits(pPCData);

    switch (pPCData->pc_state_internal)
    {
        case PCS_STARTUP_PERIPHERALS:           // Fire up all peripherals used by this power controller
              
            Drv_PowerControllers_LaunchADC();   // Start ADC Module; Note if ADC Module is already
                                                // turned on, this routine will be skipped
              
            pPCData->ftkLaunchPeripherals();    // custom function to fire up the peripherals

            //status.flags.op_status = SEPIC_STAT_OFF; // Set SEPIC status to OFF
            buckPC_GotoState(pPCData, PCS_WAIT_FOR_POWER_IN_GOOD);           //Goto next state
            break;
        
        // In this step the soft-start procedure continues with counting up until the defined power-on
        // delay period has expired.
        // PWM1H is kept low, while PWM1L is kept high to pre-charge the half-bridge bootstrap cap.
        // At the end of this phase, PWM1 output user overrides are disabled and the control is enabled.
        case PCS_WAIT_FOR_POWER_IN_GOOD:
            if (pPCData->ftkFaultDetected())
            {
                pPCData->timeCounter = 0;       //reset counter if there is a fault pending
                break;
            }
            if(++pPCData->timeCounter >= pPCData->powerInputOk_waitTime_100us)
            {
                pPCData->ftkEnableControlLoop();    // custom function to start the control loop
                buckPC_GotoState(pPCData, PCS_WAIT_FOR_ADC_ACTIVE);
            }
            break;
                 
        case PCS_WAIT_FOR_ADC_ACTIVE:               // wait until the control loop is running
            if (pPCData->flags.bits.adc_active)     // check if control loop is running
            {
                buckPC_GotoState(pPCData, PCS_RAMP_UP_VOLTAGE);
            }
            break;

        case PCS_RAMP_UP_VOLTAGE: // Increasing the voltage reference for buck by ramp_step every scheduler cycle
        {
            uint16_t newReference;

            if (pPCData->ftkFaultDetected())
                buckPC_GotoState(pPCData, PCS_SHUTDOWN);
            else
            {
                newReference = pPCData->voltageRef_compensator + pPCData->voltageRef_rampStep;
                if (newReference < pPCData->voltageRef_softStart)
                {
                    Buck_Compensator_SetOutputVoltageRef(pPCData, newReference);
                }
                else
                {
                    Buck_Compensator_SetOutputVoltageRef(pPCData, pPCData->voltageRef_softStart);
                    buckPC_GotoState(pPCData, PCS_WAIT_FOR_POWER_OUT_GOOD);
                }
            }
            break; 
        }

        case PCS_WAIT_FOR_POWER_OUT_GOOD:    // wait some time for the caps to charge and the power to be stable
            if (pPCData->ftkFaultDetected())
                buckPC_GotoState(pPCData, PCS_SHUTDOWN);
            else if (++pPCData->timeCounter >= pPCData->powerOutputOk_waitTime_100us)
            {
                buckPC_GotoState(pPCData, PCS_UP_AND_RUNNING);
            }
            break;
            
        case PCS_UP_AND_RUNNING:        // Soft start is complete, system is running, nothing to do
            if (pPCData->ftkFaultDetected())
                buckPC_GotoState(pPCData, PCS_SHUTDOWN);
            break;

        case PCS_SHUTDOWN:              // shutting down the output by setting a new voltage reference
            Buck_Compensator_SetOutputVoltageRef(pPCData, 0);
            pPCData->ftkDisableControlLoop();                   // Shutting down PWM and compensator
            if (pPCData->ftkFaultDetected() == false)
                buckPC_GotoState(pPCData, PCS_WAIT_FOR_POWER_IN_GOOD);
            break;

        default: // If something is going wrong, reset entire PWR controller
            buckPC_GotoState(pPCData, PCS_STARTUP_PERIPHERALS);
            break;
    }
}


static inline void Drv_PowerControllerBuck_MonitorVoltageLimits(POWER_CONTROLLER_DATA_t* pPCData)
{
    if (pPCData->voltageOutput > pPCData->OverVoltageLimit)
        pPCData->flags.bits.overvoltage_fault = true;
    else
        pPCData->flags.bits.overvoltage_fault = false;
    if (pPCData->voltageOutput < pPCData->UnderVoltageLimit)
        pPCData->flags.bits.undervoltage_fault = true;
    else
        pPCData->flags.bits.undervoltage_fault = false;
}

