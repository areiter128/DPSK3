/*
 * File:   init_pwm.c
 * Author: M91406
 *
 * Created on July 9, 2019, 8:39 AM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "init_pwm.h"
#include "init_gpio.h"

volatile uint16_t softstart_counter     = 0;
volatile uint16_t SoftStartDC           = FINAL_DUTY_CYCLE; // This effectively disables the openloop soft start routine
volatile uint16_t SoftStartFinished     = 0;

volatile uint16_t init_pwm_module(void) {

    
    
}

volatile uint16_t init_buck_pwm(void) {

    
}

volatile uint16_t launch_buck_pwm(void) {
    
    
}

volatile uint16_t init_boost_pwm(void) {

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
    PG2IOCONLbits.OVRDAT = 0b00; // Data for PWMxH/PWMxL Pins if Override Event is Active: PWMxL=OVRDAT0, PWMxH=OVRDAR1
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

volatile uint16_t launch_boost_pwm(void) {
    
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

void __attribute__((__interrupt__, no_auto_psv)) _PWM1Interrupt(void)
{
    if (++softstart_counter == SST_PERIOD) 
    {
        softstart_counter = 0;
        
        if (++SoftStartDC <= FINAL_DUTY_CYCLE) 
        {
            PG1DC++;
            PG2DC++;
            while (PG1STATbits.UPDATE | PG2STATbits.UPDATE);
            PG1STATbits.UPDREQ = 1; // Update all PWM registers
            PG2STATbits.UPDREQ = 1; // Update all PWM registers
        }
        else {
            IEC4bits.PWM1IE         = 0;    // Disabling PWM1 interrupt when FINAL_DUTY_CYCLE is reached
            SoftStartFinished       = 1; 
            
            // ToDo: For debug only
            PG3EVTHbits.IEVTSEL     = 0b10;     // ADC TRIGGER 1 event interrupts CPU
            IPC17bits.PWM3IP        = 6;        // Setting PWM1 interrupt priority
            IFS4bits.PWM3IF         = 0;        // Clearing PWM3 interrupt flag 
            IEC4bits.PWM3IE         = 0;        // Disable PWM3 interrupt
            
            PG4EVTHbits.IEVTSEL     = 0b10;     // ADC TRIGGER 1 event interrupts CPU
            IPC17bits.PWM4IP        = 6;        // Setting PWM1 interrupt priority
            IFS4bits.PWM4IF         = 0;        // Clearing PWM4 interrupt flag 
            IEC4bits.PWM4IE         = 0;        // Disable PWM4 interrupt
                       
        }
    }
    
    IFS4bits.PWM1IF     = 0;                // Clearing PWM1 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _PWM3Interrupt(void)
{
    DBGPIN_1_SET;
//    DBGPIN_1_TOGGLE;                // Toggle DEBUG-PIN
    DBGPIN_1_CLEAR;
    IFS4bits.PWM3IF     = 0;    // Clearing PWM3 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _PWM4Interrupt(void)
{
    DBGPIN_1_SET;
//    DBGPIN_1_TOGGLE;                // Toggle DEBUG-PIN
    DBGPIN_1_CLEAR;
    IFS4bits.PWM4IF     = 0;    // Clearing PWM4 interrupt flag
}

    
// This PWM is used only to generate synchronized ADC Trigger 1 for the buck converter
volatile uint16_t init_buck_trig_pwm(void) {

    
}

volatile uint16_t launch_buck_trig_pwm(void) {
    
    Nop();
    Nop();
    Nop();
        
    PG3CONLbits.ON = 1; // PWM Generator #2 Enable: PWM Generator is enabled
    while(PG3STATbits.UPDATE);
    PG3STATbits.UPDREQ = 1; // Update all PWM registers

    PG3IOCONHbits.PENH = 0; // PWMxH Output Port Enable: Disabled
    PG3IOCONHbits.PENL = 0; // PWMxL Output Port Enable: Disabled
    PG3IOCONLbits.OVRENH = 0;  // User Override Enable for PWMxH Pin: User override disabled
    PG3IOCONLbits.OVRENL = 0;  // User Override Enable for PWMxL Pin: User override disabled
   
    return(1); 
}

// This PWM is used only to generate synchronized ADC Trigger 1 for the boost converter
volatile uint16_t init_boost_trig_pwm(void) {

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

volatile uint16_t launch_boost_trig_pwm(void) {
  
    Nop();
    Nop();
    Nop();
    

    
    PG4CONLbits.ON = 1;         // PWM Generator #2 Enable: PWM Generator is enabled
    PG4STATbits.UPDREQ = 1;     // Update all PWM registers

    PG4IOCONHbits.PENH = 0;     // PWMxH Output Port Enable: Disabled
    PG4IOCONHbits.PENL = 0;     // PWMxL Output Port Enable: Disabled
    PG4IOCONLbits.OVRENH = 0;   // User Override Enable for PWMxH Pin: User override disabled
    PG4IOCONLbits.OVRENL = 0;   // User Override Enable for PWMxL Pin: User override disabled


    
    return(1); 
}