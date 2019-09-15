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

#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_buck_generic.h"
#include "driver/power_controllers/drv_power_controller_buck_custom.h"
#include "driver/power_controllers/npnz16b.h"
#include "driver/power_controllers/c2p2z_buck.h"

//=======================================================================================================
// local defines
//=======================================================================================================
// Buck instance 1 specific defines:
#define BUCK1_ADC_REFERENCE     3.3             // 3.3 Volts ==> maximum ADC-Value
#define BUCK1_ADC_RESOLUTION    4095UL          // 12 bits
#define BUCK1_FEEDBACK_GAIN     0.5             // 1k /(1k+1k)
#define BUCK1_IIN_FEEDBACK_GAIN 1.0             // 1 V/A

#define INIT_DACDATH_BUCK       0  // DAC value for the buck the slope starts from
#define INIT_DACDATL_BUCK       0  // Set this to minimum in Slope mode

#define BUCK_MIN_PCMC_CLAMP     0  // [A]; Minimum clamping value for buck converter input current
#define BUCK_MAX_PCMC_CLAMP     2  // [A]; Maximum clamping value for buck converter input current 

#define BUCK_VRF             (uint16_t)(BUCK1_VREF * BUCK1_IIN_FEEDBACK_GAIN * BUCK1_ADC_RESOLUTION / BUCK1_ADC_REFERENCE)
#define BUCK_MIN_PCMC_CL     (uint16_t)(BUCK_MIN_PCMC_CLAMP * BUCK1_IIN_FEEDBACK_GAIN * BUCK1_ADC_RESOLUTION / BUCK1_ADC_REFERENCE)
#define BUCK_MAX_PCMC_CL     (uint16_t)(BUCK_MAX_PCMC_CLAMP * BUCK1_IIN_FEEDBACK_GAIN * BUCK1_ADC_RESOLUTION / BUCK1_ADC_REFERENCE)
#define BUCK_RP_VREF_PER     (uint16_t)((BUCK1_VREF_RAMPUP_PERIOD / MAIN_EXECUTION_PERIOD)-1.0)
#define BUCK_RP_VREF_STEP    (uint16_t)((BUCK_VRF)/(BUCK_RP_VREF_PER + 1))
#define BUCK_DAC_SLOPE_RATE  (uint16_t)((16.0 * (BUCK1_SLEW_RATE / DAC_GRAN) / (1.0e-6/DACCLK)) + 1.0) // SLOPE DATA in [DAC-ticks/CLK-tick]

POWER_CONTROLLER_DATA_t pwrCtrlBuck1_Data;      // data instance for the buck converter


//=======================================================================================================
// @brief   returns the Output Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllerBuck1_GetOutputVoltage()
{
    return (double)(((unsigned long)pwrCtrlBuck1_Data.voltageOutput * BUCK1_ADC_REFERENCE) / (BUCK1_FEEDBACK_GAIN * BUCK1_ADC_RESOLUTION));
}


//=======================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//=======================================================================================================
void Drv_PowerControllerBuck1_SetOutputVoltageReference(double newVoltRef)
{
    pwrCtrlBuck1_Data.voltageRef_softStart = ((newVoltRef * BUCK1_ADC_RESOLUTION * BUCK1_FEEDBACK_GAIN) / BUCK1_ADC_REFERENCE);
}


//=======================================================================================================
// @brief   sets the Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage
//=======================================================================================================
void Drv_PowerControllerBuck1_SetOutputVoltageReference_mV(uint32_t newVoltRef_mV)
{
    pwrCtrlBuck1_Data.voltageRef_softStart = ((((uint32_t)newVoltRef_mV * BUCK1_ADC_RESOLUTION) * BUCK1_FEEDBACK_GAIN) / BUCK1_ADC_REFERENCE) / 1000;
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

void Drv_PowerControllerBuck1_LaunchPeripherals(void)
{
    Drv_PowerControllerBuck1_LaunchACMP();          // Start analog comparator/DAC module
    Drv_PowerControllerBuck1_LaunchAuxiliaryPWM();  // Start auxiliary PWM 
    Drv_PowerControllerBuck1_LaunchPWM();           // Start PWM
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
            
    pwrCtrlBuck1_Data.voltageRef_rampStep = BUCK_RP_VREF_STEP;              // N adc values per 100µs
    if(pwrCtrlBuck1_Data.voltageRef_rampStep == 0) {         // Protecting startup settings against 
        pwrCtrlBuck1_Data.voltageRef_rampStep = 1;           // ZERO settings
    }
                        pwrCtrlBuck1_Data.powerInputOk_waitTime_100us = 1000;   // 100ms input power stabilization delay
    pwrCtrlBuck1_Data.powerOutputOk_waitTime_100us = 1000;  // 100ms output power stabilization delay
    pwrCtrlBuck1_Data.ftkEnableControlLoop  = Drv_PowerControllerBuck1_EnableControlLoop;
    pwrCtrlBuck1_Data.ftkDisableControlLoop = Drv_PowerControllerBuck1_DisableControlLoop;
    pwrCtrlBuck1_Data.ftkLaunchPeripherals  = Drv_PowerControllerBuck1_LaunchPeripherals;
    
    Drv_PowerControllerBuck1_InitAuxiliaryPWM(); // Set up auxiliary PWM 
    Drv_PowerControllerBuck1_InitPWM();          // Set up primary PWM 
    Drv_PowerControllerBuck1_InitACMP();         // Set up comparator/DAC for PCMC
    Drv_PowerControllerBuck1_InitADC();          // Set up ADC (voltage feedback only)
   
    c2p2z_buck_Init();
    c2p2z_buck.ADCTriggerOffset = VOUT_ADCTRIG;
    c2p2z_buck.ptrADCTriggerRegister = &PG3TRIGA;
    c2p2z_buck.InputOffset = 0;
    c2p2z_buck.ptrControlReference = &(pwrCtrlBuck1_Data.voltageRef_compensator);    //TODO: wrong pointer!
    c2p2z_buck.ptrSource = &ADCBUF13;
    c2p2z_buck.ptrTarget = &DAC1DATH;
    c2p2z_buck.MaxOutput = BUCK_MAX_PCMC_CL;
    c2p2z_buck.MinOutput = BUCK_MIN_PCMC_CL;
    c2p2z_buck.status.flag.enable = 0;
}

volatile uint16_t Drv_PowerControllerBuck1_InitPWM(void)
{
    // Initialize PWMx GPIOs
    LATBbits.LATB14 = 0;    // Set GPIO RB14 LOW (PWM1H)
    TRISBbits.TRISB14 = 0;  // Make GPIO RB14 an output (PWM1H)
    CNPDBbits.CNPDB14 = 1;  // Enable intern pull down register (PWM1H)
    
    LATBbits.LATB15 = 0;    // Set GPIO RB15 LOW (PWM1L)
    TRISBbits.TRISB15 = 0;  // Make GPIO RB15 an output (PWM1L)
    CNPDBbits.CNPDB15 = 1;  // Enable intern pull down register (PWM1L)

    // PWM GENERATOR x CONTROL REGISTERS
    PG1CONLbits.ON = 0; // PWM Generator #1 Enable: PWM Generator is not enabled
    PG1CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG1CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator 1
    PG1CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG1CONLbits.MODSEL = 0b001; // PWM Mode Selection: Independent Edge PWM mode
    
    PG1CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG1CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG1CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG1CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG1CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG1CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG1CONHbits.SOCS = 0; // Start-of-Cycle Selection: Local EOC, PWM Generator is self-triggered

    // ************************
    // ToDo: CHECK IF THIS SETTING IS CORRET AND DEAD TIMES ARE STILL INSERTED CORRECTLY
    PG1IOCONLbits.CLMOD = 0;    // If PCI current limit is active, then the CLDAT[1:0] bits define the PWM output levels
    // ************************

    PG1IOCONLbits.SWAP = 0;    // Swap PWM Signals to PWMxH and PWMxL Device Pins: PWMxH/L signals are mapped to their respective pins
    PG1IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG1IOCONLbits.OVRENL = 1;  // User Override Enable for PWMxL Pin: OVRDAT0 provides data for output on the PWMxL pin
    PG1IOCONLbits.OVRDAT = 0b01; // Data for PWMxH/PWMxL Pins if Override Event is Active: PWMxL=OVRDAT0, PWMxH=OVRDAT1
    PG1IOCONLbits.OSYNC = 0b00; // User Output Override Synchronization Control: User output overrides via the OVRENH/L and OVRDAT[1:0] bits are synchronized to the local PWM time base (next Start-of-Cycle)
    
    PG1IOCONLbits.FLTDAT = 0b00; // Data for PWMxH/PWMxL Pins if Fault Event is Active: PWMxL=FLTDAT0, PWMxH=FLTDAR1
    PG1IOCONLbits.CLDAT = 0b01; // Data for PWMxH/PWMxL Pins if Current-Limit Event is Active: PWMxL=CLDAT0, PWMxH=CLDAR1
    PG1IOCONLbits.FFDAT = 0b00; // Data for PWMxH/PWMxL Pins if Feed-Forward Event is Active: PWMxL=CLDAT0, PWMxH=CLDAR1
    PG1IOCONLbits.DBDAT = 0b00; // Data for PWMxH/PWMxL Pins if Debug Mode Event is Active: PWMxL=DBDAT0, PWMxH=DBDAR1

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG1IOCONHbits.CAPSRC = 0b000;  // Time Base Capture Source Selection: No hardware source selected for time base capture ? software only
    PG1IOCONHbits.DTCMPSEL = 0; // Dead-Time Compensation Selection: Dead-time compensation is controlled by PCI Sync logic
    PG1IOCONHbits.PMOD = 0b00; // PWM Generator Output Mode Selection: PWM Generator outputs operate in Complementary mode
    PG1IOCONHbits.PENH = 0; // PWMxH Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxH output pin
    PG1IOCONHbits.PENL = 0; // PWMxL Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxL output pin
    PG1IOCONHbits.POLH = 0; // PWMxH Output Port Enable: Output pin is active-high
    PG1IOCONHbits.POLL = 0; // PWMxL Output Port Enable: Output pin is active-high
    
    // PWM GENERATOR x STATUS REGISTER
    PG1STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR x EVENT REGISTER LOW 
    PG1EVTLbits.ADTR1PS     = 0b00000;      // ADC Trigger 1 Postscaler Selection = 1:1
    PG1EVTLbits.ADTR1EN1    = 0b0;          // PG1TRIGA  Compare Event is disabled as trigger source for ADC Trigger 1 
    PG1EVTLbits.ADTR1EN2    = 0b1;          // PG1TRIGB  Compare Event is enabled as trigger source for ADC Trigger 1 -> Slope start trigger
    PG1EVTLbits.ADTR1EN3    = 0b0;          // PG1TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG1EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG1STAT<4>) manually
    PG1EVTLbits.PGTRGSEL    = 0b000;        // EOC event is the PWM Generator trigger
    
    // PWM GENERATOR x EVENT REGISTER HIGH
    PG1EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG1EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG1EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG1EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG1EVTHbits.IEVTSEL     = 0b11;         // Time base interrupts are disabled
    PG1EVTHbits.ADTR2EN1    = 0b0;          // PG1TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG1EVTHbits.ADTR2EN2    = 0b0;          // PG1TRIGB register compare event is disabled as trigger source for ADC Trigger 2 
    PG1EVTHbits.ADTR2EN3    = 0b1;          // PG1TRIGC register compare event is enabled as trigger source for ADC Trigger 2 -> Slope stop trigger
    PG1EVTHbits.ADTR1OFS    = 0b00000;      // ADC Trigger 1 offset = No offset
    
    // PGCLPCIH: PWM GENERATOR CL PCI REGISTER HIGH
    PG1CLPCIHbits.BPEN      = 0b0;          // PCI function is not bypassed
    PG1CLPCIHbits.BPSEL     = 0b000;        // PCI control is sourced from PWM Generator 1 PCI logic when BPEN = 1
    PG1CLPCIHbits.ACP       = 0b011;        // PCI Acceptance Mode: Latched
    PG1CLPCIHbits.SWPCI     = 0b0;          // Drives a '0' to PCI logic assigned to by the SWPCIM<1:0> control bits
    PG1CLPCIHbits.SWPCIM    = 0b00;         // SWPCI bit is assigned to PCI acceptance logic
    PG1CLPCIHbits.PCIGT     = 0b1;          // SR latch is Reset-dominant in Latched Acceptance modes
    PG1CLPCIHbits.TQPS      = 0b1;          // Termination Qualifier (0= not inverted, 1= inverted)
    PG1CLPCIHbits.TQSS      = 0b100;        // Termination Qualifier Source: PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
    
    // PGCLPCIL: PWM GENERATOR CL PCI REGISTER LOW
    PG1CLPCILbits.TSYNCDIS  = 0;            // Termination of latched PCI occurs at PWM EOC
    PG1CLPCILbits.TERM      = 0b001;        // Termination Event: Terminate when Comparator 1 output transitions from active to inactive
    PG1CLPCILbits.AQPS      = 0b0;          // Acceptance Qualifier signal is non-inverted 
    PG1CLPCILbits.AQSS      = 0b100;        // Acceptance Qualifier: PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits) 
    PG1CLPCILbits.SWTERM    = 0b0;          // A write of '1' to this location will produce a termination event. This bit location always reads as '0'.
    PG1CLPCILbits.PSYNC     = 0;            // PCI source is not synchronized to PWM EOC
    PG1CLPCILbits.PPS       = 0;            // Non-inverted PCI polarity
    PG1CLPCILbits.PSS       = 0b11011;      // Selecting Comparator 1 output as PCI input
    
    // Reset further PCI control registers
    PG1FPCIH        = 0x0000;          // PWM GENERATOR F PCI REGISTER HIGH
    PG1FPCIL        = 0x0000;          // PWM GENERATOR F PCI REGISTER LOW
    PG1FFPCIH       = 0x0000;          // PWM GENERATOR FF PCI REGISTER HIGH
    PG1FFPCIL       = 0x0000;          // PWM GENERATOR FF PCI REGISTER LOW
    PG1SPCIH        = 0x0000;          // PWM GENERATOR S PCI REGISTER HIGH
    PG1SPCIL        = 0x0000;          // PWM GENERATOR S PCI REGISTER LOW
    
    // PWM GENERATOR x LEADING-EDGE BLANKING REGISTER HIGH 
    PG1LEBHbits.PWMPCI      = 0b010;        // PWM Generator #3 output is made available to PCI logic
    PG1LEBHbits.PHR         = 0b1;          // Rising edge of PWM1H will trigger the LEB duration counter
    PG1LEBHbits.PHF         = 0b0;          // LEB ignores the falling edge of PWM1H
    PG1LEBHbits.PLR         = 0b0;          // LEB ignores the rising edge of PWM1L
    PG1LEBHbits.PLF         = 0b0;          // LEB ignores the falling edge of PWM1L
    
    // PWM GENERATOR x LEADING-EDGE BLANKING REGISTER LOW 
    PG1LEBL     = PWM_LEB_PERIOD;   // ToDo: This value may needs further adjustment
    
    // PGxPHASE: PWM GENERATOR x PHASE REGISTER
    PG1PHASE    = 0;
    
    // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
    PG1DC       = MAX_DUTY_CYCLE;
    
    // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER
    PG1DCA      =  0x0000;      
    
    // PGxPER: PWM GENERATOR x PERIOD REGISTER        
    PG1PER      = 0;     // Master defines the period

    // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    PG1TRIGA    = 0;  
    
    // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER       
    PG1TRIGB    = SLP_TRIG_START;   // Defining start of slope; ToDo: Check this value on oscilloscope
    
    // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER        
    PG1TRIGC    = SLP_TRIG_STOP;    // Defining end of slope;  ToDo: Check this value on oscilloscope  
    
    // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW        
    PG1DTL      = PWM_DEAD_TIME_FALLING;
    
    // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    PG1DTH      = PWM_DEAD_TIME_RISING;
            
//  PG1CAP      = 0x0000;   // Read only register
   
    return(1);
}

volatile uint16_t Drv_PowerControllerBuck1_LaunchPWM(void)
{
    Nop();
    Nop();
    Nop();
    
    PG1CONLbits.ON = 1; // PWM Generator #1 Enable: PWM Generator is enabled
    
    while(PG1STATbits.UPDATE);
    PG1STATbits.UPDREQ = 1; // Update all PWM registers

    PG1IOCONHbits.PENH = 1; // PWMxH Output Port Enable: PWM generator controls the PWMxH output pin
    PG1IOCONHbits.PENL = 1; // PWMxL Output Port Enable: PWM generator controls the PWMxL output pin
    
    return(1);
}

volatile uint16_t Drv_PowerControllerBuck1_InitAuxiliaryPWM(void)
{
    // PWM GENERATOR x CONTROL REGISTERS
    PG3CONLbits.ON = 0; // PWM Generator #3 Enable: PWM Generator is not enabled
    PG3CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG3CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator x
    PG3CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG3CONLbits.MODSEL = 0b001; // PWM Mode Selection: Variable Phase PWM mode
    
    PG3CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG3CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG3CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG3CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG3CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG3CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG3CONHbits.SOCS = 1; // Start-of-Cycle Selection: Trigger output selected by PG1

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER LOW
    PG3IOCONL = 0x0000;
    
    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG3IOCONHbits.CAPSRC = 0b000;  // Time Base Capture Source Selection: No hardware source selected for time base capture ? software only
    PG3IOCONHbits.DTCMPSEL = 0; // Dead-Time Compensation Selection: Dead-time compensation is controlled by PCI Sync logic
    PG3IOCONHbits.PMOD = 0b01; // PWM Generator Output Mode Selection: PWM Generator outputs operate in Independent mode
    PG3IOCONHbits.PENH = 0; // PWMxH Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxH output pin
    PG3IOCONHbits.PENL = 0; // PWMxL Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxL output pin
    PG3IOCONHbits.POLH = 0; // PWMxH Output Port Enable: Output pin is active-high
    PG3IOCONHbits.POLL = 0; // PWMxL Output Port Enable: Output pin is active-high
    
    // PWM GENERATOR x STATUS REGISTER
    PG3STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR x EVENT REGISTER LOW 
    PG3EVTLbits.ADTR1PS     = 0b00001;      // ADC Trigger 1 Postscaler Selection = 1:2
    PG3EVTLbits.ADTR1EN1    = 0b1;          // PG3TRIGA  Compare Event is enabled as trigger source for ADC Trigger 1
    PG3EVTLbits.ADTR1EN2    = 0b0;          // PG3TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.ADTR1EN3    = 0b0;          // PG3TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG1STAT<4>) manually
    PG3EVTLbits.PGTRGSEL    = 0b011;        // PGxTRIGC compare event is the PWM Generator trigger => This serves as SOC trigger for PG2 (Boost Converter)
    
    // PWM GENERATOR x EVENT REGISTER HIGH
    PG3EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG3EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG3EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG3EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG3EVTHbits.IEVTSEL     = 0b11;         // Interrupt Event Selection: Time base interrupts are disabled
    PG3EVTHbits.ADTR2EN1    = 0b0;          // PG3TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR2EN2    = 0b0;          // PG3TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR2EN3    = 0b0;          // PG3TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR1OFS    = 0b00000;      // ADC Trigger 1 offset = No offset
    
    // PCI function for current limitation is not used
    PG3CLPCIH       = 0x0000;           // PWM GENERATOR CL PCI REGISTER HIGH
    PG3CLPCIL       = 0x0000;           // PWM GENERATOR CL PCI REGISTER LOW
      
    // Reset further PCI control registers
    PG3FPCIH        = 0x0000;          // PWM GENERATOR F PCI REGISTER HIGH
    PG3FPCIL        = 0x0000;          // PWM GENERATOR F PCI REGISTER LOW
    PG3FFPCIH       = 0x0000;          // PWM GENERATOR FF PCI REGISTER HIGH
    PG3FFPCIL       = 0x0000;          // PWM GENERATOR FF PCI REGISTER LOW
    PG3SPCIH        = 0x0000;          // PWM GENERATOR S PCI REGISTER HIGH
    PG3SPCIL        = 0x0000;          // PWM GENERATOR S PCI REGISTER LOW
    
    // Leading edge blanking is not used
    PG3LEBH         = 0x0000;
    PG3LEBL         = 0x0000;
    
        
    // PGxPHASE: PWM GENERATOR x PHASE REGISTER
    PG3PHASE    = PWM_AUX_PHASE_SHIFT;
    
    // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
    PG3DC       = (MAX_DUTY_CYCLE - PWM_AUX_PHASE_SHIFT);     
    
    // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER
    PG3DCA      =  0x0000;      
    
    // PGxPER: PWM GENERATOR x PERIOD REGISTER        
    PG3PER      = 0;     // Master defines the period

    // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    PG3TRIGA    = VOUT_ADCTRIG;  // ToDo: Check this value on oscilloscope
    
    // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER       
    PG3TRIGB    = 0;  
    
    // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER        
    PG3TRIGC    = BOOST_OFFSET;  
    
    // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW        
    PG3DTL      = 0;
    
    // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    PG3DTH      = 0;
            
//  PG3CAP      = 0x0000;   // Read only register
   
    return(1);
}

volatile uint16_t Drv_PowerControllerBuck1_LaunchAuxiliaryPWM(void)
{
    Nop();
    Nop();
    Nop();
        
    PG3CONLbits.ON = 1; // PWM Generator #3 Enable: PWM Generator is enabled
    
    while(PG3STATbits.UPDATE);
    PG3STATbits.UPDREQ = 1; // Update all PWM registers

    PG3IOCONHbits.PENH = 0; // PWMxH Output Port Enable: Disabled
    PG3IOCONHbits.PENL = 0; // PWMxL Output Port Enable: Disabled
    PG3IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: User override disabled
    PG3IOCONLbits.OVRENL = 1;  // User Override Enable for PWMxL Pin: User override disabled
   
    return(1); 
}

volatile uint16_t Drv_PowerControllerBuck1_InitACMP(void)
{
    // DACxCONL: DACx CONTROL LOW REGISTER
    DAC1CONLbits.DACEN = 0; // Individual DACx Module Enable: Disables DACx module during configuration
    DAC1CONLbits.IRQM = 0b00; // Interrupt Mode Selection: Interrupts are disabled
    DAC1CONLbits.CBE = 1; // Comparator Blank Enable: Enables the analog comparator output to be blanked (gated off) during the recovery transition following the completion of a slope operation
    DAC1CONLbits.DACOEN = 0; // DACx Output Buffer Enable: DACx analog voltage is connected to the DACOUT1 pin (RA3/TP35 on DPSK3)
    // DAC1CONLbits.CMPSTAT (read only bit)
    
    // Comparator filter and hysteresis options
    DAC1CONLbits.FLTREN = 0; // Comparator Digital Filter Enable: Digital filter is disabled
    DAC1CONLbits.CMPPOL = 0; // Comparator Output Polarity Control: Output is non-inverted
    DAC1CONLbits.INSEL = 0b000; // Comparator Input Source Select: feedback is connected to CMPxA input pin
    DAC1CONLbits.HYSPOL = 0; // Comparator Hysteresis Polarity Selection: Hysteresis is applied to the rising edge of the comparator output
    DAC1CONLbits.HYSSEL = 0b01; // Comparator Hysteresis Selection: 45 mv hysteresis (0 = 0mV, 1 = 15mV, 2 = 30mV, 3 = 45mV)
    
    // DACxCONH: DACx CONTROL HIGH REGISTER
    
    // ***********************************************
    // ToDo: CHECK DAC LEB PERIOD TO BE CORRECT AND DOESN'T CREATE CONFLICTS
    DAC1CONHbits.TMCB = DAC_TMCB; // DACx Leading-Edge Blanking: period for the comparator
    // ***********************************************
        
    // DACxDATH: DACx DATA HIGH REGISTER
    DAC1DATH = (INIT_DACDATH_BUCK & 0x0FFF); // DACx Data: This register specifies the high DACx data value. Valid values are from 205 to 3890.
    DAC1DATL = (INIT_DACDATL_BUCK & 0x0FFF); // DACx Low Data
        
    // SLPxCONH: DACx SLOPE CONTROL HIGH REGISTER
    SLP1CONHbits.SLOPEN = 1; // Slope Function Enable/On: Enables slope function
    SLP1CONHbits.HME = 0; // Hysteretic Mode Enable: Disables Hysteretic mode for DACx
    SLP1CONHbits.TWME = 0; // Triangle Wave Mode Enable: Disables Triangle Wave mode for DACx
    SLP1CONHbits.PSE = 0; // Positive Slope Mode Enable: Slope mode is negative (decreasing)
    
    // SLPxCONL: DACx SLOPE CONTROL LOW REGISTER
    SLP1CONLbits.HCFSEL = 0b0000; // Hysteretic Comparator Function Input Selection: (none)
    SLP1CONLbits.SLPSTOPA = 0b0001; // Slope Stop A Signal Selection: PWM1 Trigger 2
    SLP1CONLbits.SLPSTOPB = 0b0000; // Slope Stop B Signal Selection: (none, ramp always resets at max duty cycle)
    SLP1CONLbits.SLPSTRT = 0b0001; // Slope Start Signal Selection: PWM1 Trigger 1
    
    // ToDo: CHECK SLP1DAT in conjunction with DAC1DATH and DAC1DATL
    // DAC1DATL should be reserved/valid only in hysteretic and triangular mode
    // So for normal slope compensation the valid registers should be DAC1DATH as reference level
    // and SLP1DAT for the slew rate (V/usec translated in DAC-ticks/time-ticks)
    // Previous configurations have shown that this might not be true, so please revisit this setting.
    
    // SLPxDAT: DACx SLOPE DATA REGISTER
    SLP1DAT = BUCK_DAC_SLOPE_RATE; // Slope Ramp Rate Value
        
    return(1);
}

volatile uint16_t Drv_PowerControllerBuck1_LaunchACMP(void)
{
   DAC1CONLbits.DACEN = 1; // Individual DACx Module Enable: Enables DAC1 module 
   DACCTRL1Lbits.DACON = 1; // Common DAC Module Enable: Enables all enabled DAC modules
    
   return(1); 
}

volatile uint16_t Drv_PowerControllerBuck1_InitADC(void)
{
   // ANSELx: ANALOG SELECT FOR PORTx REGISTER
    ANSELCbits.ANSELC1 = 1; // Analog input is enabled and digital input is disabled for RC1 (Buck converter output voltage feedback)
    
    // ADLVLTRGL: ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGLbits.LVLEN13 = 0; // Input trigger is edge-sensitive

    // ADMOD0L: ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD0Hbits.DIFF13 = 0; // Differential-Mode for Corresponding Analog Inputs: Channel is single-ended
    ADMOD0Hbits.SIGN13 = 0; // Output Data Sign for Corresponding Analog Inputs: Channel output data are unsigned
    
    // ADEIEL: ADC EARLY INTERRUPT ENABLE REGISTER LOW
    ADEIELbits.EIEN13 = 1; // Early interrupt is enabled for the channel
    
    // ADIEL: ADC INTERRUPT ENABLE REGISTER LOW
    ADIELbits.IE13 = 1; // Common Interrupt Enable: Common and individual interrupts are disabled for the corresponding channel
    
    // ADTRIGnL/ADTRIGnH: ADC CHANNEL TRIGGER n(x) SELECTION REGISTERS LOW AND HIGH
    ADTRIG3Lbits.TRGSRC13 = 0b01000; // Trigger Source Selection for Corresponding Analog Inputs: PWM3 Trigger 1
    
    // ADCMPxCON: ADC DIGITAL COMPARATOR x CONTROL REGISTER
    ADCMP1CONbits.CHNL = 13; // Input Channel Number: 13=AN13
    ADCMP1CONbits.CMPEN = 0; // Comparator Enable: Comparator is disabled
    ADCMP1CONbits.IE = 0; // Comparator Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the comparator
    ADCMP1CONbits.BTWN = 0; // Between Low/High Comparator Event: Disabled
    ADCMP1CONbits.HIHI = 0; // High/High Comparator Event: Disabled
    ADCMP1CONbits.HILO = 0; // High/Low Comparator Event: Disabled
    ADCMP1CONbits.LOHI = 0; // Low/High Comparator Event: Disabled
    ADCMP1CONbits.LOLO = 0; // Low/Low Comparator Event: Disabled
   
    // ADCMPxENL: ADC DIGITAL COMPARATOR x CHANNEL ENABLE REGISTER LOW
    ADCMP1ENLbits.CMPEN13 = 0; // Comparator Enable for Corresponding Input Channels: AN13 Disabled
    
    // ADCMPxLO: ADC COMPARARE REGISTER LOWER THRESHOLD VALUE REGISTER
    ADCMP1LO = 620; // R1=1kOhm, R2=1kOhm, G=0.5000; 1Vin=620 ADC ticks

    // ADCMPxHI: ADC COMPARARE REGISTER UPPER THRESHOLD VALUE REGISTER
    ADCMP1HI = 2358; // R1=1kOhm, R2=1kOhm, G=0.5000; 3.8Vin=2358 ADC ticks
    
    // ADFLxCON: ADC DIGITAL FILTER x CONTROL REGISTER
    ADFL1CONbits.FLEN = 0; // Filter Enable: Filter is disabled
    ADFL1CONbits.MODE = 0b11; // Filter Mode: Averaging mode (always 12-bit result 7 in oversampling mode 12-16bit wide)
    ADFL1CONbits.OVRSAM = 0b001; // Filter Averaging/Oversampling Ratio: 16x (result in the ADFLxDAT)
    ADFL1CONbits.IE = 0; // Filter Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the filter
    ADFL1CONbits.FLCHSEL = 13; // Oversampling Filter Input Channel Selection: 13=AN13

    return(1); 
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

