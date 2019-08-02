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

#include <stdbool.h>
#include <stdint.h>
#include <xc.h>         // include processor specific defines and functions

#include "driver/drv_buck_power_controller.h"
#include "init_pwm.h"
#include "init_acmp.h"
#include "init_adc.h"
#include "npnz16b.h"
#include "c2p2z_buck.h"

//======================================================================================================================
// local defines
//======================================================================================================================
// ADC parameters
#define ADCREFERENCE        3.3
#define ADCRESOLUTION     4095UL
//buck feedback gains   
#define VBUCKFBGAIN   0.5   // 1k /(1k+1k)

typedef enum
{
    BUCK_STATE_STARTUP_PERIPHERALS      = 1,    // Fire up all the peripherals that are involved
    BUCK_STATE_POWER_ON_DELAY           = 2,    // Soft-Start Power On Delay
    BUCK_STATE_WAIT_FOR_ACTIVE_ADC      = 3,    // wait until ADC is running
    BUCK_STATE_RAMP_UP_VOLTAGE          = 4,    // Soft-Start Ramp Up Output Voltage
    BUCK_STATE_WAIT_TO_STABILIZE        = 5,    // Soft-Start wait to stabilize
    BUCK_STATE_MONITOR_OUTPUT_VOLTAGE   = 6     // Soft-Start is complete
}BUCK_SM_STATES_e;

volatile uint16_t           buckPC_VoltageOutput;
volatile uint16_t           buckPC_VoltageReference;                // 2047 for 3.3V
volatile uint16_t           buckPC_VoltageReferenceCompensator;     // 2047 for 3.3V
volatile uint16_t           buckPC_OverVoltage;
volatile uint16_t           buckPC_UnderVoltage;
volatile uint16_t           buckPC_VoltageReferenceRampStep;
volatile uint16_t           buckPC_PowerOn_DelayTime;
volatile uint16_t           buckPC_PowerOn_StabilizationTime;
volatile uint16_t           buckPC_State_TimeCounter;
volatile BUCK_SM_STATES_e   buckPC_State = BUCK_STATE_STARTUP_PERIPHERALS;
volatile bool               buckPC_ADC_is_active;
volatile bool               buckPC_OverVoltageFlag;
volatile bool               buckPC_UnderVoltageFlag;

//======================================================================================================================
// @brief   returns the raw ADC value for the Output Voltage
//======================================================================================================================
uint16_t Drv_BuckPowerController_GetOutputVoltageRaw(void)
{
    return buckPC_VoltageOutput;
}

//======================================================================================================================
// @brief   returns the Output Voltage in Volts as a double
//======================================================================================================================
double Drv_BuckPowerController_GetOutputVoltage(void)
{
    return (double)(((unsigned long)buckPC_VoltageOutput * ADCREFERENCE) / (VBUCKFBGAIN * ADCRESOLUTION));
}

//======================================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//======================================================================================================================
void Drv_BuckPowerController_SetOutputVoltageReference(double newVoltRef)
{
    buckPC_VoltageReference = newVoltRef * (VBUCKFBGAIN * ADCRESOLUTION) / ADCREFERENCE;
    buckPC_OverVoltage  = (uint32_t) (buckPC_VoltageReference * BUCK_OVERVOLTAGE_PERCENT) / 100;
    buckPC_UnderVoltage = (uint32_t) (buckPC_VoltageReference * BUCK_UNDERVOLTAGE_PERCENT) / 100;
}


//======================================================================================================================
// @brief   internal helping function for the state machine to set the new state
// @note    the also resets the timer automatically
//======================================================================================================================
void buckPC_GotoState(BUCK_SM_STATES_e newState)
{
    buckPC_State_TimeCounter = 0;
    buckPC_State = newState;
}

//======================================================================================================================
// @brief   Initializes all peripherals and data structures of the buck controller like PWN, ADC, DAC, CMP etc.
// @note    call this during booting up the system before you call anything else or the Power Controller
//======================================================================================================================
void Drv_BuckPowerController_Init(void)
{
    init_buck_trig_pwm();   // Set up auxiliary PWM for buck converter
    init_buck_pwm();        // Set up buck converter PWM
    init_buck_acmp();       // Set up buck converter peak current comparator/DAC
    init_buck_adc();        // Set up buck converter ADC (voltage feedback only)
    
    Drv_BuckPowerController_SetOutputVoltageReference(0.0); //Init with 0 Volt
    buckPC_VoltageReferenceCompensator = 0;                 //Init with 0 Volt
    buckPC_VoltageReferenceRampStep = 4;        // 4 adc values per 100µs
    buckPC_PowerOn_DelayTime = 1000;            // Soft-Start Power-On Delay = 100 ms
    buckPC_PowerOn_StabilizationTime = 1000;    // Soft-Start Stabilization Delay = 100 ms
    buckPC_ADC_is_active = false;               // reset flag
    buckPC_OverVoltageFlag = false;
    buckPC_UnderVoltageFlag = false;
    
    c2p2z_buck_Init();
    c2p2z_buck.ADCTriggerOffset = VOUT_ADC_TRIGGER_DELAY;
    c2p2z_buck.ptrADCTriggerRegister = &PG3TRIGA;
    c2p2z_buck.InputOffset = 0;
    c2p2z_buck.ptrControlReference = &buckPC_VoltageReferenceCompensator;
    c2p2z_buck.ptrSource = &ADCBUF13;
    c2p2z_buck.ptrTarget = &DAC1DATH;
    c2p2z_buck.MaxOutput = 3600;
    c2p2z_buck.MinOutput = 10;
    c2p2z_buck.status.flag.enable = 0;
    
    // Reset state machine to the beginning
    buckPC_GotoState(BUCK_STATE_STARTUP_PERIPHERALS);
}


//======================================================================================================================
// @brief   Task that runs all the necessary things to do like soft start and voltage monitoring
// @note    call this every 100us from your main scheduler to ensure the right timing
//======================================================================================================================
void Drv_BuckPowerController_Task_100us(void)
{
    //TODO Monitor Over Undervoltage every time before calling the statemachine
    switch (buckPC_State)
    {
        // Fire up all peripherals used by this power controller
        case BUCK_STATE_STARTUP_PERIPHERALS:
            launch_adc();                                   // Start ADC Module
            launch_buck_acmp();                             // Start analog comparator/DAC module
            launch_buck_trig_pwm();                         // Start auxiliary PWM 
            launch_buck_pwm();                              // Start PWM
            buckPC_GotoState(BUCK_STATE_POWER_ON_DELAY);    //Goto next state
            break;
        
        // In this step the soft-start procedure continues with counting up until the defined power-on
        // delay period has expired.
        // PWM1H is kept low, while PWM1L is kept high to pre-charge the half-bridge bootstrap cap.
        // At the end of this phase, PWM1 output user overrides are disabled and the control is enabled.
        case BUCK_STATE_POWER_ON_DELAY:
            if(++buckPC_State_TimeCounter >= buckPC_PowerOn_DelayTime)
            {
                PG1IOCONLbits.OVRENH = 0;           // User override disabled for PWMxH Pin
                PG1IOCONLbits.OVRENL = 0;           // User override disabled for PWMxL Pin
                buckPC_GotoState(BUCK_STATE_WAIT_FOR_ACTIVE_ADC);
            }
            break;
                 
        case BUCK_STATE_WAIT_FOR_ACTIVE_ADC:    // wait until the power controller is active
            if (buckPC_ADC_is_active)
            {
                c2p2z_buck.status.flag.enable = 1;  // Start the control loop for buck
                buckPC_GotoState(BUCK_STATE_RAMP_UP_VOLTAGE);
            }
            break;
                 
        case BUCK_STATE_RAMP_UP_VOLTAGE: // Increasing reference for buck by 4 every scheduler cycle
        {
            uint16_t tmpVoltRef = buckPC_VoltageReferenceCompensator + buckPC_VoltageReferenceRampStep;
            if (tmpVoltRef < buckPC_VoltageReference)
            {
                buckPC_VoltageReferenceCompensator = tmpVoltRef;
            }
            else
            {
                buckPC_VoltageReferenceCompensator = buckPC_VoltageReference;
                buckPC_GotoState(BUCK_STATE_WAIT_TO_STABILIZE);
            }
            break; 
        }

        case BUCK_STATE_WAIT_TO_STABILIZE:    // wait until the power controller is active
            if (buckPC_State_TimeCounter >= buckPC_PowerOn_StabilizationTime)
            {
                buckPC_GotoState(BUCK_STATE_MONITOR_OUTPUT_VOLTAGE);
            }
            break;
                 
        case BUCK_STATE_MONITOR_OUTPUT_VOLTAGE:   // Soft start is complete, system is running, nothing to do
            if (buckPC_VoltageOutput >= buckPC_OverVoltage)
                buckPC_OverVoltageFlag = true;
            else
                buckPC_OverVoltageFlag = false;
            if (buckPC_VoltageOutput <= buckPC_UnderVoltage)
                buckPC_UnderVoltageFlag = true;
            else
                buckPC_UnderVoltageFlag = false;
            break;

        default: // If something is going wrong, reset entire PWR controller
            buckPC_GotoState(BUCK_STATE_STARTUP_PERIPHERALS);
            break;
    }
}


//======================================================================================================================
// @brief   Interrupt routine for calling the buck c2p2z compensator and sampling the Output Voltage
//======================================================================================================================
void __attribute__((__interrupt__, auto_psv)) _ADCAN13Interrupt(void)
{
    c2p2z_buck_Update(&c2p2z_buck);     //call the compensator as soon as possible
    // the readout of the ADC register is mandatory to make the reset of the interrupt flag stick
    // if we would not read from the ADC register then the interrupt flag would be set immediately after resetting it
    buckPC_VoltageOutput =  ADCBUF13;
    buckPC_ADC_is_active = true;
    _ADCAN13IF = 0;  // Clear the ADCANx interrupt flag. read from ADCBUFx first to make it stick
}


