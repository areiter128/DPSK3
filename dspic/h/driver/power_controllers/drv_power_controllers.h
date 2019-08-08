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
// @file drv_power_controllers.h
//
// @brief contains the generic and custom parts of the power controller(s)
//
// @version v1.0
// @date 2019-08-06
// @author M52409
//
//=======================================================================================================

#ifndef _DRV_POWER_CONTROLLERS_H_
#define _DRV_POWER_CONTROLLERS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PCS_STARTUP_PERIPHERALS      = 1,    // Fire up all the peripherals that are involved
    PCS_STANDBY                  = 2,    // Soft-Start Standby (wait for GO command)
    PCS_WAIT_FOR_POWER_IN_GOOD   = 3,    // Soft-Start wait some time to guarantee a stable power supply
    PCS_WAIT_FOR_ADC_ACTIVE      = 4,    // wait until ADC is running
    PCS_RAMP_UP_VOLTAGE          = 5,    // Soft-Start Ramp Up Output Voltage
    PCS_WAIT_FOR_POWER_OUT_GOOD  = 6,    // Soft-Start wait to stabilize
    PCS_UP_AND_RUNNING           = 7     // Soft-Start is complete, power is up and running
}PWR_CTRL_STATE_INT_e;


typedef enum
{
    STATE_OFF     = 0b000,  // PwrCtrl Status Off:      Everything is inactive incl. peripherals
    STATE_STANDBY = 0b001,  // PwrCtrl Status Standby:  Peripherals are running but controller and PWM outputs are off
    STATE_START   = 0b010,  // PwrCtrl Status Startup:  Converter is executing its startup procedure
    STATE_ON      = 0b011,  // PwrCtrl Status On:       Power Controller is Active and Running
    STATE_FAULT   = 0b100   // PwrCtrl Status Fault:    Power Controller has been shut down waiting for restart attempt
} PWR_CTRL_STATE_e;

typedef struct
{
    volatile PWR_CTRL_STATE_e op_status :3; // Bit <0:2> operation status
    volatile unsigned : 5;                              // Bit <3:7> (reserved)
    volatile bool pwm_active  :1;                       // Bit  8: Status bit indicating that the PWM outputs have been enabled
    volatile bool adc_active  :1;                       // Bit  9: Status bit indicating that the ADC has been started and is sampling data
    volatile bool fault_active  :1;                     // Bit 10: Status bit indicating that a critical fault condition has been detected
    volatile bool overvoltage_fault  :1;                // Bit 11: Status bit indicating that a critical fault condition has been detected
    volatile bool undervoltage_fault  :1;               // Bit 12: Status bit indicating that a critical fault condition has been detected
    volatile bool GO :1;                                // Bit 13: POWER SUPPLY START bit (will trigger startup procedure when set)
    volatile bool auto_start :1;                        // Bit 14: Auto-Start will automatically enable the converter and set the GO bit when ready
    volatile bool enabled :1;                           // Bit 15: Enable-bit (when disabled, power supply will reset in STANDBY mode)
} __attribute__((packed))POWER_CONTROLLER_FLAGBITS_t;

typedef union {
	volatile uint16_t value;                 // buffer for 16-bit word read/write operations
	volatile POWER_CONTROLLER_FLAGBITS_t bits; // data structure for single bit addressing operations
} POWER_CONTROLLER_FLAGS_t;                  // SEPIC operation status bits

typedef void (*pCallback_t)(void);

typedef struct
{
    //TODO: is the reference always voltage or sometimes current?
    volatile uint16_t powerInputOk_waitTime_100us;  // amount of 100�s ticks to wait for the input power to be stable
    volatile uint16_t powerOutputOk_waitTime_100us; // amount of 100�s ticks to wait for the output power to be stable
//    volatile uint16_t precharge_delay;      // Soft-Start Bootstrap Capacitor pre-charge delay if necessary
//    volatile uint16_t rampUp_period;            // Soft-Start Ramp-Up Duration
    volatile uint16_t timeCounter;              // Soft-Start Execution Counter
    volatile PWR_CTRL_STATE_INT_e pc_state_internal;   // state of the power control state machine
    volatile uint16_t voltageOutput;            // Output Voltage measured by the ADC
    volatile uint16_t voltageRef_softStart;     // target voltage reference value for soft start
    volatile uint16_t voltageRef_compensator;   // voltage reference for the compensator
    volatile uint16_t voltageRef_rampStep;      // Soft-Start Single Reference Increment per Step
    volatile uint16_t OverVoltageLimit;         // Overvoltage Limit, gets calculated automatically
    volatile uint16_t UnderVoltageLimit;        // Overvoltage Limit, gets calculated automatically
    volatile POWER_CONTROLLER_FLAGS_t flags;    // status flags
    volatile pCallback_t ftkEnableControlLoop;  // Controller calls this function to enable the control Loop
    volatile pCallback_t ftkDisableControlLoop; // Controller calls this function to disable the control Loop
}POWER_CONTROLLER_DATA_t;                       // power control soft-start settings and variables

extern void Drv_PowerControllers_Init(void);
extern void Drv_PowerControllers_Task_100us(void);

#endif  //_DRV_POWER_CONTROLLERS_H_
