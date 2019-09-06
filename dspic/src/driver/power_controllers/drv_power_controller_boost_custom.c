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
// @file drv_power_controller_boost_custom.c
//
// @brief power controller functions for boost converter
//
// @author M91406
// @author M52409
// @author M91281
//
// @date September 4, 2019, 2:45 PM
//=======================================================================================================

#include <stdbool.h>
#include <stdint.h>
#include "xc.h"

#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_boost_generic.h"
#include "driver/power_controllers/drv_power_controller_boost_custom.h"
#include "driver/power_controllers/npnz16b.h"
#include "driver/power_controllers/c2P2Z_boost.h"

//=======================================================================================================
// local defines
//=======================================================================================================
// Boost instance 1 specific defines:
#define BOOST1_ADC_REFERENCE     3.3             // 3.3 Volts ==> maximum ADC-Value
#define BOOST1_ADC_RESOLUTION    4095UL          // 12 bits
#define BOOST1_FEEDBACK_BGAIN    0.1253          // 1k /(1k+6.98k)

POWER_CONTROLLER_DATA_t pwrCtrlBoost1_Data;      // data instance for the boost converter


//=======================================================================================================
// @brief   returns the Output Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllerBoost1_GetOutputVoltage()
{
    return (double)(((unsigned long)pwrCtrlBoost1_Data.voltageOutput * BOOST1_ADC_REFERENCE) / (BOOST1_FEEDBACK_BGAIN * BOOST1_ADC_RESOLUTION));
}

//=======================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//=======================================================================================================
void Drv_PowerControllerBoost1_SetOutputVoltageReference(double newVoltRef)
{
    pwrCtrlBoost1_Data.voltageRef_softStart = ((newVoltRef * BOOST1_ADC_RESOLUTION * BOOST1_FEEDBACK_BGAIN) / BOOST1_ADC_REFERENCE);
}

//=======================================================================================================
// @brief   sets the Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage
//=======================================================================================================
void Drv_PowerControllerBoost1_SetOutputVoltageReference_mV(uint32_t newVoltRef_mV)
{
    pwrCtrlBoost1_Data.voltageRef_softStart = ((((uint32_t)newVoltRef_mV * BOOST1_ADC_RESOLUTION) * BOOST1_FEEDBACK_BGAIN) / BOOST1_ADC_REFERENCE) / 1000;
}

void Drv_PowerControllerBoost1_EnableControlLoop(void)
{
    c2P2Z_boost.status.flag.enable = 1;  // Start the control loop for boost
    PG2IOCONLbits.OVRENH = 0;           // User override disabled for PWMxH Pin
    PG2IOCONLbits.OVRENL = 0;           // User override disabled for PWMxL Pin
}

void Drv_PowerControllerBoost1_DisableControlLoop(void)
{
    c2P2Z_boost.status.flag.enable = 0;  // Stop the control loop for boost
    PG2IOCONLbits.OVRENH = 1;           // User override disabled for PWMxH Pin
    PG2IOCONLbits.OVRENL = 1;           // User override disabled for PWMxL Pin
    //TODO: should we set some output pin for safety reasons???
}

void Drv_PowerControllerBoost1_LaunchPeripherals(void)
{
    Drv_PowerControllerBoost1_LaunchACMP();          // Start analog comparator/DAC module
    Drv_PowerControllerBoost1_LaunchAuxiliaryPWM();  // Start auxiliary PWM 
    Drv_PowerControllerBoost1_LaunchPWM();           // Start PWM
}

//=======================================================================================================
// @brief   Initializes the Boost Power Converter - Instance 1
// @note    In this routine all the application specific custom functions are implemented
//=======================================================================================================
void Drv_PowerControllerBoost1_Init(bool autostart)
{
    Drv_PowerControllerBoost_Init(&pwrCtrlBoost1_Data, autostart);

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
            
    pwrCtrlBoost1_Data.voltageRef_rampStep = 4;              // 4 adc values per 100µs
    pwrCtrlBoost1_Data.powerInputOk_waitTime_100us = 1000;   // 100ms input power stabilization delay
    pwrCtrlBoost1_Data.powerOutputOk_waitTime_100us = 1000;  // 100ms output power stabilization delay
    pwrCtrlBoost1_Data.ftkEnableControlLoop  = Drv_PowerControllerBoost1_EnableControlLoop;
    pwrCtrlBoost1_Data.ftkDisableControlLoop = Drv_PowerControllerBoost1_DisableControlLoop;
    pwrCtrlBoost1_Data.ftkLaunchPeripherals  = Drv_PowerControllerBoost1_LaunchPeripherals;
    
    Drv_PowerControllerBoost1_InitAuxiliaryPWM(); // Set up auxiliary PWM 
    Drv_PowerControllerBoost1_InitPWM();          // Set up primary PWM 
    Drv_PowerControllerBoost1_InitACMP();         // Set up comparator/DAC for PCMC
    Drv_PowerControllerBoost1_InitADC();          // Set up ADC (voltage feedback only)
   
    c2P2Z_boost_Init();
    c2P2Z_boost.ADCTriggerOffset = VOUT_ADC_TRIGGER_DELAY;
    c2P2Z_boost.ptrADCTriggerRegister = &PG4TRIGA;
    c2P2Z_boost.InputOffset = 0;
    c2P2Z_boost.ptrControlReference = &(pwrCtrlBoost1_Data.voltageRef_compensator);    //TODO: wrong pointer!
    c2P2Z_boost.ptrSource = &ADCBUF18;
    c2P2Z_boost.ptrTarget = &DAC2DATH;
    c2P2Z_boost.MaxOutput = 3600;
    c2P2Z_boost.MinOutput = 10;
    c2P2Z_boost.status.flag.enable = 0;
}

volatile uint16_t Drv_PowerControllerBoost1_InitPWM(void)
{
   // Initialize PWMx GPIOs
    LATBbits.LATB12 = 0;    // Set GPIO RB12 LOW (PWM2H)
//    TRISBbits.TRISB12 = 1;  // Make GPIO RB12 an input (PWM2H)
    CNPDBbits.CNPDB12 = 0;  // Disable intern pull down register (PWM2H)
    
    LATBbits.LATB13 = 0;    // Set GPIO RB13 LOW (PWM2L)
    TRISBbits.TRISB13 = 0;  // Make GPIO RB13 an output (PWM2L)
    CNPDBbits.CNPDB13 = 1;  // Enable intern pull down register (PWM2L)

    // PWM GENERATOR x CONTROL REGISTERS
    PG2CONLbits.ON = 0; // PWM Generator #2 Enable: PWM Generator is not enabled
    PG2CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG2CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator 2 
    PG2CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG2CONLbits.MODSEL = 0b001; // PWM Mode Selection: Variable Phase PWM mode
    
    PG2CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG2CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG2CONHbits.MPHSEL = 1; // Master Phase Register Selection: PWM Generator uses MPHASE register
    PG2CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG2CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG2CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG2CONHbits.SOCS = 1; // Start-of-Cycle Selection: PG2 is triggered by PG1

    
    // ************************
    // ToDo: CHECK IF THIS SETTING IS CORRET AND DEAD TIMES ARE STILL INSERTED CORRECTLY
    PG2IOCONLbits.CLMOD = 0;    // If PCI current limit is active, then the CLDAT[1:0] bits define the PWM output levels
    // ************************

    PG2IOCONLbits.SWAP = 0;    // Swap PWM Signals to PWMxH and PWMxL Device Pins
    PG2IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG2IOCONLbits.OVRENL = 1;  // User Override Enable for PWMxL Pin: OVRDAT0 provides data for output on the PWMxL pin
    PG2IOCONLbits.OVRDAT = 0b00; // Data for PWMxH/PWMxL Pins if Override Event is Active: PWMxL=OVRDAT0, PWMxH=OVRDAT1
    PG2IOCONLbits.OSYNC = 0b00; // User Output Override Synchronization Control: User output overrides via the OVRENH/L and OVRDAT[1:0] bits are synchronized to the local PWM time base (next Start-of-Cycle)
    
    PG2IOCONLbits.FLTDAT = 0b00; // Data for PWMxH/PWMxL Pins if Fault Event is Active: PWMxL=FLTDAT0, PWMxH=FLTDAR1
    PG2IOCONLbits.CLDAT = 0b00; // Data for PWMxH/PWMxL Pins if Current-Limit Event is Active: PWMxL=CLDAT0, PWMxH=CLDAR1
    PG2IOCONLbits.FFDAT = 0b00; // Data for PWMxH/PWMxL Pins if Feed-Forward Event is Active: PWMxL=CLDAT0, PWMxH=CLDAR1
    PG2IOCONLbits.DBDAT = 0b00; // Data for PWMxH/PWMxL Pins if Debug Mode Event is Active: PWMxL=DBDAT0, PWMxH=DBDAR1

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG2IOCONHbits.CAPSRC = 0b000;  // Time Base Capture Source Selection: No hardware source selected for time base capture: software only
    PG2IOCONHbits.DTCMPSEL = 0; // Dead-Time Compensation Selection: Dead-time compensation is controlled by PCI Sync logic
    PG2IOCONHbits.PMOD = 0b01; // PWM Generator Output Mode Selection: PWM Generator outputs operate in Independent mode
    PG2IOCONHbits.PENH = 0; // PWMxH Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxH output pin
    PG2IOCONHbits.PENL = 0; // PWMxL Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxL output pin
    PG2IOCONHbits.POLH = 0; // PWMxH Output Port Enable: Output pin is active-high
    PG2IOCONHbits.POLL = 0; // PWMxL Output Port Enable: Output pin is active-high
    
    // PWM GENERATOR x STATUS REGISTER
    PG2STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR x EVENT REGISTER LOW 
    PG2EVTLbits.ADTR1PS     = 0b00000;      // ADC Trigger 1 Postscaler Selection = 1:1
    PG2EVTLbits.ADTR1EN3    = 0b0;          // PG2TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG2EVTLbits.ADTR1EN2    = 0b0;          // PG2TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG2EVTLbits.ADTR1EN1    = 0b1;          // PG2TRIGA  Compare Event is enabled as trigger source for ADC Trigger 1 -> Slope start trigger
    PG2EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG2STAT<4>) manually
    PG2EVTLbits.PGTRGSEL    = 0b000;        // PWM Generator Trigger Output is not used, leave it as default: EOC event is the PWM Generator trigger
    
    // PWM GENERATOR x EVENT REGISTER HIGH
    PG2EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG2EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG2EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG2EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG2EVTHbits.IEVTSEL     = 0b11;         // Time base interrupts are disabled
    PG2EVTHbits.ADTR2EN3    = 0b0;          // PG2TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG2EVTHbits.ADTR2EN2    = 0b1;          // PG2TRIGB register compare event is enabled as trigger source for ADC Trigger 2 -> Slope stop trigger
    PG2EVTHbits.ADTR2EN1    = 0b0;          // PG2TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG2EVTHbits.ADTR1OFS    = 0b00000;      // ADC Trigger 1 offset = 0
    
    // PGCLPCIH: PWM GENERATOR CL PCI REGISTER HIGH
    PG2CLPCIHbits.BPEN      = 0b0;          // PCI function is not bypassed
    PG2CLPCIHbits.BPSEL     = 0b000;        // PCI control is sourced from PWM Generator 1 PCI logic when BPEN = 1
    PG2CLPCIHbits.ACP       = 0b011;        // PCI Acceptance Mode: Latched
    PG2CLPCIHbits.SWPCI     = 0b0;          // Drives a '0' to PCI logic assigned to by the SWPCIM<1:0> control bits
    PG2CLPCIHbits.SWPCIM    = 0b00;         // SWPCI bit is assigned to PCI acceptance logic
    PG2CLPCIHbits.PCIGT     = 0b0;          // SR latch is Set-dominant in Latched Acceptance modes; ToDo: Why is this not called LATMOD?
    PG2CLPCIHbits.TQPS      = 0b0;          // Termination Qualifier not inverted
    PG2CLPCIHbits.TQSS      = 0b000;        // No termination qualifier used so terminator will work straight away without any qualifier
    
    // PGCLPCIL: PWM GENERATOR CL PCI REGISTER LOW
    PG2CLPCILbits.TSYNCDIS  = 0;            // Termination of latched PCI occurs at PWM EOC
    PG2CLPCILbits.TERM      = 0b001;        // Termination Event: Terminate when Comparator 2 output transitions from active to inactive
    PG2CLPCILbits.AQPS      = 0b1;          // Acceptance Qualifier (LEB) signal is inverted 
    PG2CLPCILbits.AQSS      = 0b010;        // Acceptance Qualifier: LEB 
    PG2CLPCILbits.SWTERM    = 0b0;          // A write of '1' to this location will produce a termination event. This bit location always reads as '0'.
    PG2CLPCILbits.PSYNC     = 0;            // PCI source is not synchronized to PWM EOC
    PG2CLPCILbits.PPS       = 0;            // Non-inverted PCI polarity
    PG2CLPCILbits.PSS       = 0b11100;      // Selecting Comparator 2 output as PCI input
//     PG2CLPCILbits.PSS       = 0b00000;      // PCI is DISABLED
    
    // Reset further PCI control registers
    PG2FPCIH        = 0x0000;          // PWM GENERATOR F PCI REGISTER HIGH
    PG2FPCIL        = 0x0000;          // PWM GENERATOR F PCI REGISTER LOW
    PG2FFPCIH       = 0x0000;          // PWM GENERATOR FF PCI REGISTER HIGH
    PG2FFPCIL       = 0x0000;          // PWM GENERATOR FF PCI REGISTER LOW
    PG2SPCIH        = 0x0000;          // PWM GENERATOR S PCI REGISTER HIGH
    PG2SPCIL        = 0x0000;          // PWM GENERATOR S PCI REGISTER LOW
    
    // PWM GENERATOR x LEADING-EDGE BLANKING REGISTER HIGH 
    PG2LEBHbits.PWMPCI      = 0b000;        // PWM Generator #1 output is made available to PCI logic
    PG2LEBHbits.PHR         = 0b1;          // Rising edge of PWM2H will trigger the LEB duration counter
    PG2LEBHbits.PHF         = 0b0;          // LEB ignores the falling edge of PWM2H
    PG2LEBHbits.PLR         = 0b0;          // LEB ignores the rising edge of PWM2L
    PG2LEBHbits.PLF         = 0b0;          // LEB ignores the falling edge of PWM2L
    
    // PWM GENERATOR x LEADING-EDGE BLANKING REGISTER LOW 
    PG2LEBL                 = 52;           // ToDo: This value may need further adjustment
    
    // PG2PHASE: PWM GENERATOR x PHASE REGISTER
    PG2PHASE    = 0;
    
    // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
//    PG2DC       = 800;      // 80%
    PG2DC       = INIT_DUTY_CYCLE;      // 8% - This is the initial value for the soft-start 
    
    // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER
    PG2DCA      =  0x0000;      
    
    // PGxPER: PWM GENERATOR x PERIOD REGISTER        
    PG2PER      = 0;     // Master defines the period

    // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    PG2TRIGA    = BOOST_OFFSET + SLOPE_START_DELAY;  // Defining start of slope; ToDo: Check this value on oscilloscope
    
    // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER       
    PG2TRIGB    = BOOST_OFFSET + SLOPE_STOP_DELAY;  // Defining end of slope
    
    // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER        
    PG2TRIGC    = 0;  
    
    // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW        
    PG2DTL      = 0;
    
    // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    PG2DTH      = 0;
            
//  PG2CAP      = 0x0000;   // Read only register
    
    return(1); 
}

volatile uint16_t Drv_PowerControllerBoost1_LaunchPWM(void)
{
    Nop();
    Nop();
    Nop();
    
    PG2CONLbits.ON = 1; // PWM Generator #2 Enable: PWM Generator is enabled
    PG2STATbits.UPDREQ = 1; // Update all PWM registers

    PG2IOCONHbits.PENH = 0; // PWMxH Output Port Enable: Disabled
    PG2IOCONHbits.PENL = 1; // PWMxL Output Port Enable: PWM generator controls the PWMxL output pin
    PG2IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG2IOCONLbits.OVRENL = 0;  // User Override Enable for PWMxL Pin: User override disabled

    return(1);
}

volatile uint16_t Drv_PowerControllerBoost1_InitAuxiliaryPWM(void)
{
    // PWM GENERATOR x CONTROL REGISTERS
    PG4CONLbits.ON = 0; // PWM Generator #4 Enable: PWM Generator is not enabled
    PG4CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG4CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator x
    PG4CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG4CONLbits.MODSEL = 0b001; // PWM Mode Selection: Variable Phase PWM mode
    
    PG4CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG4CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG4CONHbits.MPHSEL = 1; // Master Phase Register Selection: PWM Generator uses MPHASE register
    PG4CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG4CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG4CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG4CONHbits.SOCS = 1; // Start-of-Cycle Selection: PG1 triggers PG4

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER LOW
    PG4IOCONL = 0x0000;
    
    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG4IOCONH = 0x0000; // GPIO registers TRISx, LATx, Rxx registers control the PWMxH and PWMxL output pins
        
    // PWM GENERATOR x STATUS REGISTER
    PG4STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR x EVENT REGISTER LOW 
    PG4EVTLbits.ADTR1PS     = 0b00001;      // ADC Trigger 1 Postscaler Selection = 1:2
    PG4EVTLbits.ADTR1EN3    = 0b0;          // PG1TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG4EVTLbits.ADTR1EN2    = 0b0;          // PG1TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG4EVTLbits.ADTR1EN1    = 0b1;          // PG1TRIGA  Compare Event is enabled as trigger source for ADC Trigger 1
    PG4EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG1STAT<4>) manually
    PG4EVTLbits.PGTRGSEL    = 0b000;        // PWM Generator Trigger Output is EOC (not used in this case)
    
    // PWM GENERATOR x EVENT REGISTER HIGH
    PG4EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG4EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG4EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG4EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG4EVTHbits.IEVTSEL     = 0b00;         // Interrupt Event Selection: Time base interrupts are disabled
    PG4EVTHbits.ADTR2EN3    = 0b0;          // PG1TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR2EN2    = 0b0;          // PG1TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR2EN1    = 0b0;          // PG1TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR1OFS    = 0b00000;      // ADC Trigger 1 offset = no offset 
    
    // Reset PCI control registers
    PG4CLPCIH       = 0x0000;           // PWM GENERATOR CL PCI REGISTER HIGH
    PG4CLPCIL       = 0x0000;           // PWM GENERATOR CL PCI REGISTER LOW
    PG4FPCIH        = 0x0000;          // PWM GENERATOR F PCI REGISTER HIGH
    PG4FPCIL        = 0x0000;          // PWM GENERATOR F PCI REGISTER LOW
    PG4FFPCIH       = 0x0000;          // PWM GENERATOR FF PCI REGISTER HIGH
    PG4FFPCIL       = 0x0000;          // PWM GENERATOR FF PCI REGISTER LOW
    PG4SPCIH        = 0x0000;          // PWM GENERATOR S PCI REGISTER HIGH
    PG4SPCIL        = 0x0000;          // PWM GENERATOR S PCI REGISTER LOW
    
    // Leading edge blanking is not used
    PG4LEBH         = 0x0000;
    PG4LEBL         = 0x0000;
    
        
    // PGxPHASE: PWM GENERATOR x PHASE REGISTER
    PG4PHASE    = 0;
    
    // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
    PG4DC       = 800;      // 80%
    
    // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER
    PG4DCA      =  0x0000;      
    
    // PGxPER: PWM GENERATOR x PERIOD REGISTER        
    PG4PER      = 0;     // Master sets the period

    // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    PG4TRIGA    = BOOST_OFFSET + VOUT_ADC_TRIGGER_DELAY;  // ToDo: Check this value on oscilloscope
    
    // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER       
    PG4TRIGB    = 0;  
    
    // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER        
    PG4TRIGC    = 0;  
    
    // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW        
    PG4DTL      = 0;
    
    // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    PG4DTH      = 0;
            
//  PG4CAP      = 0x0000;   // Read only register
   
    return(1);
}

volatile uint16_t Drv_PowerControllerBoost1_LaunchAuxiliaryPWM(void)
{
    Nop();
    Nop();
    Nop();
    
    PG4CONLbits.ON = 1;         // PWM Generator #4 Enable: PWM Generator is enabled
    PG4STATbits.UPDREQ = 1;     // Update all PWM registers

    PG4IOCONHbits.PENH = 0;     // PWMxH Output Port Enable: Disabled
    PG4IOCONHbits.PENL = 0;     // PWMxL Output Port Enable: Disabled
    PG4IOCONLbits.OVRENH = 0;   // User Override Enable for PWMxH Pin: User override disabled
    PG4IOCONLbits.OVRENL = 0;   // User Override Enable for PWMxL Pin: User override disabled

    return(1); 
}

volatile uint16_t Drv_PowerControllerBoost1_InitACMP(void)
{
    // DACxCONL: DACx CONTROL LOW REGISTER
    DAC2CONLbits.DACEN = 0; // Individual DACx Module Enable: Disables DACx module during configuration
    DAC2CONLbits.IRQM = 0b00; // Interrupt Mode Selection: Interrupts are disabled
    DAC2CONLbits.CBE = 1; // Comparator Blank Enable: Enables the analog comparator output to be blanked (gated off) during the recovery transition following the completion of a slope operation
    DAC2CONLbits.DACOEN = 0; // DACx Output Buffer Enable: disabled for this module
    DAC2CONLbits.FLTREN = 0; // Comparator Digital Filter Enable: Digital filter is disabled
    // DAC2CONLbits.CMPSTAT (read only bit)
    DAC2CONLbits.CMPPOL = 0; // Comparator Output Polarity Control: Output is non-inverted
    DAC2CONLbits.INSEL = 0b011; // Comparator Input Source Select: feedback is connected to CMPxD input pin
    DAC2CONLbits.HYSPOL = 0; // Comparator Hysteresis Polarity Selection: Hysteresis is applied to the rising edge of the comparator output
    DAC2CONLbits.HYSSEL = 0b11; // Comparator Hysteresis Selection: 45 mv hysteresis (0 = 0mV, 1 = 15mV, 2 = 30mV, 3 = 45mV)
    
    // DACxCONH: DACx CONTROL HIGH REGISTER
    
    // ***********************************************
    // ToDo: CHECK DAC LEB PERIOD TO BE CORRECT AND DOESN'T CREATE CONFLICTS
    DAC2CONHbits.TMCB = LEB_PER_COMP; // DACx Leading-Edge Blanking: period for the comparator
    // ***********************************************
        
    // DACxDATH: DACx DATA HIGH REGISTER
    DAC2DATH = (DACDATH_BOOST & 0x0FFF); // DACx Data: This register specifies the high DACx data value. Valid values are from 205 to 3890.
    DAC2DATL = (DACDATL_BOOST & 0x0FFF); // DACx Low Data
        
    // SLPxCONH: DACx SLOPE CONTROL HIGH REGISTER
    SLP2CONHbits.SLOPEN = 1; // Slope Function Enable/On: Enables slope function
    SLP2CONHbits.HME = 0; // Hysteretic Mode Enable: Disables Hysteretic mode for DACx
    SLP2CONHbits.TWME = 0; // Triangle Wave Mode Enable: Disables Triangle Wave mode for DACx
    SLP2CONHbits.PSE = 0; // Positive Slope Mode Enable: Slope mode is negative (decreasing)
    
    // SLPxCONL: DACx SLOPE CONTROL LOW REGISTER
    SLP2CONLbits.HCFSEL = 0b0000; // Hysteretic Comparator Function Input Selection: (none)
    SLP2CONLbits.SLPSTOPA = 0b0010; // Slope Stop A Signal Selection: PWM2 Trigger 2
    SLP2CONLbits.SLPSTOPB = 0b0000; // Slope Stop B Signal Selection: (none, ramp always resets at max duty cycle)
    SLP2CONLbits.SLPSTRT = 0b0010; // Slope Start Signal Selection: PWM2 Trigger 1
    
    // ToDo: CHECK SLP2DAT in conjunction with DAC2DATH and DAC2DATL
    // DAC2DATL should be reserved/valid only in hysteretic and triangular mode
    // So for normal slope compensation the valid registers should be DAC2DATH as reference level
    // and SLP2DAT for the slew rate (V/usec translated in DAC-ticks/time-ticks)
    // Previous configurations have shown that this might not be true, so please revisit this setting.
    
    // SLPxDAT: DACx SLOPE DATA REGISTER
    SLP2DAT = SLOPE_RATE; // Slope Ramp Rate Value
     
    return(1);
}

volatile uint16_t Drv_PowerControllerBoost1_LaunchACMP(void) {
    
    DAC2CONLbits.DACEN = 1; // Individual DACx Module Enable: Enables DAC2 module 
    DACCTRL1Lbits.DACON = 1; // Common DAC Module Enable: Enables all enabled DAC modules
    
    return(1);
}

volatile uint16_t Drv_PowerControllerBoost1_InitADC(void)
{
    // ANSELx: ANALOG SELECT FOR PORTx REGISTER
    ANSELDbits.ANSELD10 = 1; // Analog input is enabled and digital input is disabled for RD10 (Boost converter output voltage feedback)
    
    // ADLVLTRGL: ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGHbits.LVLEN18 = 0; // Input trigger is edge-sensitive
    
    // ADMOD0L: ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD1Lbits.DIFF18 = 0; // Differential-Mode for Corresponding Analog Inputs: Channel is single-ended
    ADMOD1Lbits.SIGN18 = 0; // Output Data Sign for Corresponding Analog Inputs: Channel output data are unsigned
    
    // ADEIEL: ADC EARLY INTERRUPT ENABLE REGISTER LOW
    ADEIEHbits.EIEN18 = 1; // Early interrupt is enabled for the channel
    
    // ADIEL: ADC INTERRUPT ENABLE REGISTER LOW
    ADIEHbits.IE18 = 1; // Common Interrupt Enable: Common and individual interrupts are disabled for the corresponding channel

    // ADTRIGnL/ADTRIGnH: ADC CHANNEL TRIGGER n(x) SELECTION REGISTERS LOW AND HIGH
    ADTRIG4Hbits.TRGSRC18 = 0b01010; // Trigger Source Selection for Corresponding Analog Inputs: PWM4 Trigger 1
    
    // ADCMPxCON: ADC DIGITAL COMPARATOR x CONTROL REGISTER
    ADCMP2CONbits.CHNL = 18; // Input Channel Number: 13=AN13
    ADCMP2CONbits.CMPEN = 0; // Comparator Enable: Comparator is disabled
    ADCMP2CONbits.IE = 0; // Comparator Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the comparator
    ADCMP2CONbits.BTWN = 0; // Between Low/High Comparator Event: Disabled
    ADCMP2CONbits.HIHI = 0; // High/High Comparator Event: Disabled
    ADCMP2CONbits.HILO = 0; // High/Low Comparator Event: Disabled
    ADCMP2CONbits.LOHI = 0; // Low/High Comparator Event: Disabled
    ADCMP2CONbits.LOLO = 0; // Low/Low Comparator Event: Disabled
    
    // ADCMPxENL: ADC DIGITAL COMPARATOR x CHANNEL ENABLE REGISTER LOW
    ADCMP2ENHbits.CMPEN18 = 0; // Comparator Enable for Corresponding Input Channels: AN18 Disabled
    
    // ADCMPxLO: ADC COMPARARE REGISTER LOWER THRESHOLD VALUE REGISTER
    ADCMP2LO = 1399; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 9Vin=1399 ADC ticks

    // ADCMPxHI: ADC COMPARARE REGISTER UPPER THRESHOLD VALUE REGISTER
    ADCMP2HI = 2799; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 18Vin=2799 ADC ticks
    
    // ADFLxCON: ADC DIGITAL FILTER x CONTROL REGISTER
    ADFL2CONbits.FLEN = 0; // Filter Enable: Filter is disabled
    ADFL2CONbits.MODE = 0b11; // Filter Mode: Averaging mode (always 12-bit result 7 in oversampling mode 12-16bit wide)
    ADFL2CONbits.OVRSAM = 0b001; // Filter Averaging/Oversampling Ratio: 16x (result in the ADFLxDAT)
    ADFL2CONbits.IE = 0; // Filter Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the filter
    ADFL2CONbits.FLCHSEL = 18; // Oversampling Filter Input Channel Selection: 18=AN18
    
    return(1);
}

//=======================================================================================================
// @brief   Interrupt routine for calling the boost c2p2z compensator and sampling the Output Voltage
//=======================================================================================================
void __attribute__((__interrupt__, auto_psv)) _ADCAN18Interrupt(void)
{
    c2P2Z_boost_Update(&c2P2Z_boost);     //call the compensator as soon as possible
    // the readout of the ADC register is mandatory to make the reset of the interrupt flag stick
    // if we would not read from the ADC register then the interrupt flag would be set immediately after resetting it
    pwrCtrlBoost1_Data.voltageOutput =  ADCBUF18;
    pwrCtrlBoost1_Data.flags.bits.adc_active = true;
    _ADCAN18IF = 0;  // Clear the ADCANx interrupt flag. read from ADCBUFx first to make it stick
    
    //TODO: discuss, if we should call the boost_Update routine at first or after resetting the interrupt flag?
}

