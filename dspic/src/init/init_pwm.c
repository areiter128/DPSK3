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


volatile uint16_t init_pwm_module(void) {

    // Make sure power to the peripheral is enabled
    PMD1bits.PWMMD = 0; // PWM Module Disable: PWM module is enabled
    
    // PWM GENERATOR ENABLE
    PG1CONLbits.ON = 0; // PWM Generator #1 Enable: PWM Generator is not enabled
    PG2CONLbits.ON = 0; // PWM Generator #2 Enable: PWM Generator is not enabled
    PG3CONLbits.ON = 0; // PWM Generator #3 Enable: PWM Generator is not enabled
    PG4CONLbits.ON = 0; // PWM Generator #4 Enable: PWM Generator is not enabled
    PG5CONLbits.ON = 0; // PWM Generator #5 Enable: PWM Generator is not enabled
    PG6CONLbits.ON = 0; // PWM Generator #6 Enable: PWM Generator is not enabled
    PG7CONLbits.ON = 0; // PWM Generator #7 Enable: PWM Generator is not enabled
    PG8CONLbits.ON = 0; // PWM Generator #8 Enable: PWM Generator is not enabled
    
    // PWM CLOCK CONTROL REGISTER
    PCLKCONbits.LOCK = 0;       // Lock bit: Write-protected registers and bits are unlocked
    PCLKCONbits.DIVSEL = 0b00;  // PWM Clock Divider Selection: Divide ratio is 1:2
    PCLKCONbits.MCLKSEL = 0b11; // PWM Master Clock Selection: Auxiliary PLL post-divider output
    
    // FREQUENCY SCALE REGISTER & FREQUENCY SCALING MINIMUM PERIOD REGISTER
    FSCL = 0x0000;      // Reset frequency scaling register
    FSMINPER = 0x0000;  // Reset frequency scaling minimum register
    
    // MASTER PHASE, DUTY CYCLE AND PERIOD REGISTERS
    MPHASE = 0x0000;    // Reset master phase
    MDC = 0x0000;       // Reset master duty cycle
    MPER = 1000;        // Master period set to 2us
    
    // LINEAR FEEDBACK SHIFT REGISTER
    LFSR = 0x0000;      // Reset linear feedback shift register
    
    // COMBINATIONAL TRIGGER REGISTERS
    CMBTRIGLbits.CTA1EN = 0; // Disable Trigger Output from PWM Generator #1 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA2EN = 0; // Disable Trigger Output from PWM Generator #2 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA3EN = 0; // Disable Trigger Output from PWM Generator #3 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA4EN = 0; // Disable Trigger Output from PWM Generator #4 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA5EN = 0; // Disable Trigger Output from PWM Generator #5 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA6EN = 0; // Disable Trigger Output from PWM Generator #6 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA7EN = 0; // Disable Trigger Output from PWM Generator #7 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA8EN = 0; // Disable Trigger Output from PWM Generator #8 as Source for Combinational Trigger A
    
    CMBTRIGHbits.CTB1EN = 0; // Disable Trigger Output from PWM Generator #1 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB2EN = 0; // Disable Trigger Output from PWM Generator #2 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB3EN = 0; // Disable Trigger Output from PWM Generator #3 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB4EN = 0; // Disable Trigger Output from PWM Generator #4 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB5EN = 0; // Disable Trigger Output from PWM Generator #5 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB6EN = 0; // Disable Trigger Output from PWM Generator #6 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB7EN = 0; // Disable Trigger Output from PWM Generator #7 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB8EN = 0; // Disable Trigger Output from PWM Generator #8 as Source for Combinational Trigger B

    // COMBINATORIAL PWM LOGIC A CONTROL REGISTERS A-F
    LOGCONAbits.PWMS1A = 0b0000; // Combinatorial PWM Logic Source #1 Selection: PWM1H
    LOGCONAbits.S1APOL = 0;      // Combinatorial PWM Logic Source #1 Polarity: Input is positive logic
    LOGCONAbits.PWMS2A = 0b0010; // Combinatorial PWM Logic Source #2 Selection: PWM2H
    LOGCONAbits.S2APOL = 0;      // Combinatorial PWM Logic Source #2 Polarity: Input is positive logic
    LOGCONAbits.PWMLFA = 0b01;   // Combinatorial PWM Logic Function Selection: PWMS1y & PWMS2y (AND)
    LOGCONAbits.PWMLFAD = 0b000; // Combinatorial PWM Logic Destination Selection: No assignment, combinatorial PWM logic function is disabled
    
    // Reset further combinatorial logic registers
    LOGCONB = 0x0000; // LOGCONB: COMBINATORIAL PWM LOGIC CONTROL REGISTER B
    LOGCONC = 0x0000; // LOGCONC: COMBINATORIAL PWM LOGIC CONTROL REGISTER C
    LOGCOND = 0x0000; // LOGCOND: COMBINATORIAL PWM LOGIC CONTROL REGISTER D
    LOGCONE = 0x0000; // LOGCONE: COMBINATORIAL PWM LOGIC CONTROL REGISTER E
    LOGCONF = 0x0000; // LOGCONF: COMBINATORIAL PWM LOGIC CONTROL REGISTER F
    
    // PWM EVENT OUTPUT CONTROL REGISTERS A-F
    PWMEVTAbits.EVTAOEN = 0;    // PWM Event Output Enable: Event output signal is internal only
    PWMEVTAbits.EVTAPOL = 0;    // PWM Event Output Polarity: Event output signal is active-high
    PWMEVTAbits.EVTASTRD = 0;   // PWM Event Output Stretch Disable: Event output signal is stretched to eight PWM clock cycles minimum
    PWMEVTAbits.EVTASYNC = 0;   // PWM Event Output Sync: Event output is not synchronized to the system clock
    PWMEVTAbits.EVTASEL = 0b0000; // PWM Event Selection: Source is selected by the PGTRGSEL[2:0] bits
    PWMEVTAbits.EVTAPGS = 0b000;  // PWM Event Source Selection: PWM Generator 1
    
    // Reset further PWM event output registers
    PWMEVTB = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER B
    PWMEVTC = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER C
    PWMEVTD = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER D
    PWMEVTE = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER E
    PWMEVTF = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER F
    
    
    return(1);
    
}

volatile uint16_t init_buck_pwm(void) {

    // Initialize PWM1 GPIOs
    LATBbits.LATB14 = 0;    // Set GPIO RB14 LOW (PWM1H)
    TRISBbits.TRISB14 = 0;  // Make GPIO RB14 an output (PWM1H)
    CNPDBbits.CNPDB14 = 1;  // Enable intern pull down register (PWM1H)
    
    LATBbits.LATB15 = 0;    // Set GPIO RB15 LOW (PWM1L)
    TRISBbits.TRISB15 = 0;  // Make GPIO RB15 an output (PWM1L)
    CNPDBbits.CNPDB15 = 1;  // Enable intern pull down register (PWM1L)

    // PWM GENERATOR 1 CONTROL REGISTERS
    PG1CONLbits.ON = 0; // PWM Generator #1 Enable: PWM Generator is not enabled
    PG1CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG1CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator 1
    PG1CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG1CONLbits.MODSEL = 0b000; // PWM Mode Selection: Independent Edge PWM mode
    
    PG1CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG1CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG1CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG1CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG1CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG1CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG1CONHbits.SOCS = 0b0000; // Start-of-Cycle Selection: Local EOC PWM Generator is self-triggered

    // ************************
    // ToDo: CHECK IF THIS SETTING IS CORRET AND DEAD TIMES ARE STILL INSERTED CORRECTLY
    PG1IOCONLbits.CLMOD = 0;    // If PCI current limit is active, then the CLDAT[1:0] bits define the PWM output levels
    // ************************

    PG1IOCONLbits.SWAP = 0;    // Swap PWM Signals to PWMxH and PWMxL Device Pins: PWMxH/L signals are mapped to their respective pins
    PG1IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG1IOCONLbits.OVRENL = 1;  // User Override Enable for PWMxL Pin: OVRDAT0 provides data for output on the PWMxL pin
    PG1IOCONLbits.OVRDAT = 0b00; // Data for PWMxH/PWMxL Pins if Override Event is Active: PWMxL=OVRDAT0, PWMxH=OVRDAR1
    PG1IOCONLbits.OSYNC = 0b00; // User Output Override Synchronization Control: User output overrides via the OVRENH/L and OVRDAT[1:0] bits are synchronized to the local PWM time base (next Start-of-Cycle)
    
    PG1IOCONLbits.FLTDAT = 0b00; // Data for PWMxH/PWMxL Pins if Fault Event is Active: PWMxL=FLTDAT0, PWMxH=FLTDAR1
    PG1IOCONLbits.CLDAT = 0b00; // Data for PWMxH/PWMxL Pins if Current-Limit Event is Active: PWMxL=CLDAT0, PWMxH=CLDAR1
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
    
    // PWM GENERATOR 1 STATUS REGISTER
    PG1STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR 1 EVENT REGISTER LOW 
    PG1EVTLbits.ADTR1PS     = 0b00000;      // ADC Trigger 1 Postscaler Selection = 1:1 -> to be used for starting the ramp for slope compensation
    PG1EVTLbits.ADTR1EN3    = 0b0;          // PG1TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG1EVTLbits.ADTR1EN2    = 0b0;          // PG1TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG1EVTLbits.ADTR1EN1    = 0b1;          // PG1TRIGA  Compare Event is enabled as trigger source for ADC Trigger 1
    PG1EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG1STAT<4>) manually
    PG1EVTLbits.PGTRGSEL    = 0b000;        // PWM Generator Trigger Output is not used, leave it as default: EOC event is the PWM Generator trigger
    
    // PWM GENERATOR 1 EVENT REGISTER HIGH
    PG1EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG1EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG1EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG1EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG1EVTHbits.IEVTSEL     = 0b11;         // Time base interrupts are disabled
    PG1EVTHbits.ADTR2EN3    = 0b0;          // PG1TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG1EVTHbits.ADTR2EN2    = 0b0;          // PG1TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG1EVTHbits.ADTR2EN1    = 0b0;          // PG1TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG1EVTHbits.ADTR1OFS    = 0b00000;      // ADC Trigger 1 offset = No offset
    
    // PGCLPCIH: PWM GENERATOR CL PCI REGISTER HIGH
    PG1CLPCIHbits.BPEN      = 0b0;          // PCI function is not bypassed
    PG1CLPCIHbits.BPSEL     = 0b000;        // PCI control is sourced from PWM Generator 1 PCI logic when BPEN = 1
    PG1CLPCIHbits.ACP       = 0b011;        // PCI Acceptance Mode: Latched
    PG1CLPCIHbits.SWPCI     = 0b0;          // Drives a '0' to PCI logic assigned to by the SWPCIM<1:0> control bits
    PG1CLPCIHbits.SWPCIM    = 0b00;         // SWPCI bit is assigned to PCI acceptance logic
    PG1CLPCIHbits.PCIGT     = 0b0;          // SR latch is Set-dominant in Latched Acceptance modes
    PG1CLPCIHbits.TQPS      = 0b0;          // Termination Qualifier not inverted
    PG1CLPCIHbits.TQSS      = 0b000;        // No termination qualifier used so terminator will work straight away without any qualifier
    
    // PGCLPCIL: PWM GENERATOR CL PCI REGISTER LOW
    PG1CLPCILbits.TSYNCDIS  = 0;            // Termination of latched PCI occurs at PWM EOC
    PG1CLPCILbits.TERM      = 0b001;        // Termination Event: Terminate when Comparator 1 output transitions from active to inactive
    PG1CLPCILbits.AQPS      = 0b1;          // Acceptance Qualifier (LEB) signal is inverted 
    PG1CLPCILbits.AQSS      = 0b010;        // Acceptance Qualifier: LEB 
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
    
    // PWM GENERATOR 1 LEADING-EDGE BLANKING REGISTER HIGH 
    PG1LEBHbits.PWMPCI      = 0b000;        // PWM Generator #1 output is made available to PCI logic
    PG1LEBHbits.PHR         = 0b1;          // Rising edge of PWM1H will trigger the LEB duration counter
    PG1LEBHbits.PHF         = 0b0;          // LEB ignores the falling edge of PWM1H
    PG1LEBHbits.PLR         = 0b0;          // LEB ignores the rising edge of PWM1L
    PG1LEBHbits.PLF         = 0b0;          // LEB ignores the falling edge of PWM1L
    
    // PWM GENERATOR 1 LEADING-EDGE BLANKING REGISTER LOW 
    PG1LEBL                 = 52;           // ToDo: This value may need further adjustment
    
    // PG1PHASE: PWM GENERATOR 1 PHASE REGISTER
    PG1PHASE    = 0;
    
    // PG1DC: PWM GENERATOR 1 DUTY CYCLE REGISTER
    PG1DC       = 800;      // 80%
    
    // PG1DCA: PWM GENERATOR 1 DUTY CYCLE ADJUSTMENT REGISTER
    PG1DCA      =  0x0000;      
    
    // PG1PER: PWM GENERATOR 1 PERIOD REGISTER        
    PG1PER      = 1000;     // 500 kHz

    // PG1TRIGA: PWM GENERATOR 1 TRIGGER A REGISTER
    PG1TRIGA    = 100;  // ToDo: Check this value on oscilloscope
    
    // PG1TRIGB: PWM GENERATOR 1 TRIGGER B REGISTER       
    PG1TRIGB    = 150;  // ToDo: Check this value on oscilloscope
    
    // PG1TRIGC: PWM GENERATOR 1 TRIGGER C REGISTER        
    PG1TRIGC    = 800;  // ToDo: Check this value on oscilloscope
    
    // PG1DTL: PWM GENERATOR 1 DEAD-TIME REGISTER LOW        
    PG1DTL      = 40;
    
    // PG1DTH: PWM GENERATOR 1 DEAD-TIME REGISTER HIGH
    PG1DTH      = 20;
            
//  PG1CAP      = 0x0000;   // Read only register
   
    return(1);
}

volatile uint16_t launch_buck_pwm(void) {
    
    Nop();
    Nop();
    Nop();
    
    PG1CONLbits.ON = 1; // PWM Generator #1 Enable: PWM Generator is not enabled
    PG1STATbits.UPDREQ = 1; // Update all PWM registers

    PG1IOCONHbits.PENH = 1; // PWMxH Output Port Enable: PWM generator controls the PWMxH output pin
    PG1IOCONHbits.PENL = 1; // PWMxL Output Port Enable: PWM generator controls the PWMxL output pin
    PG1IOCONLbits.OVRENH = 0;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG1IOCONLbits.OVRENL = 0;  // User Override Enable for PWMxL Pin: OVRDAT0 provides data for output on the PWMxL pin

    return(1);
}

volatile uint16_t init_boost_pwm(void) {

// Initialize PWM1 GPIOs
    LATBbits.LATB12 = 0;    // Set GPIO RB12 LOW (PWM2H)
    TRISBbits.TRISB12 = 1;  // Make GPIO RB12 an input (PWM2H)
    CNPDBbits.CNPDB12 = 0;  // Disable intern pull down register (PWM2H)
    
    LATBbits.LATB13 = 0;    // Set GPIO RB13 LOW (PWM2L)
    TRISBbits.TRISB13 = 0;  // Make GPIO RB13 an output (PWM2L)
    CNPDBbits.CNPDB13 = 1;  // Enable intern pull down register (PWM2L)

    // PWM GENERATOR 1 CONTROL REGISTERS
    PG2CONLbits.ON = 0; // PWM Generator #2 Enable: PWM Generator is not enabled
    PG2CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG2CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator 2 
    PG2CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG2CONLbits.MODSEL = 0b000; // PWM Mode Selection: Independent Edge PWM mode
    
    PG2CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG2CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG2CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG2CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG2CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG2CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode    
    PG2CONHbits.SOCS = 0b0000; // Start-of-Cycle Selection: Local EOC PWM Generator is self-triggered     
        
 
    // ************************
    // ToDo: CHECK IF THIS SETTING IS CORRET AND DEAD TIMES ARE STILL INSERTED CORRECTLY
    PG2IOCONLbits.CLMOD = 0;    // If PCI current limit is active, then the CLDAT[1:0] bits define the PWM output levels
    // ************************

    PG2IOCONLbits.SWAP = 1;    // Swap PWM Signals to PWMxH and PWMxL Device Pins
    PG2IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG2IOCONLbits.OVRENL = 1;  // User Override Enable for PWMxL Pin: OVRDAT0 provides data for output on the PWMxL pin
    PG2IOCONLbits.OVRDAT = 0b00; // Data for PWMxH/PWMxL Pins if Override Event is Active: PWMxL=OVRDAT0, PWMxH=OVRDAR1
    PG2IOCONLbits.OSYNC = 0b00; // User Output Override Synchronization Control: User output overrides via the OVRENH/L and OVRDAT[1:0] bits are synchronized to the local PWM time base (next Start-of-Cycle)
    
    PG2IOCONLbits.FLTDAT = 0b00; // Data for PWMxH/PWMxL Pins if Fault Event is Active: PWMxL=FLTDAT0, PWMxH=FLTDAR1
    PG2IOCONLbits.CLDAT = 0b00; // Data for PWMxH/PWMxL Pins if Current-Limit Event is Active: PWMxL=CLDAT0, PWMxH=CLDAR1
    PG2IOCONLbits.FFDAT = 0b00; // Data for PWMxH/PWMxL Pins if Feed-Forward Event is Active: PWMxL=CLDAT0, PWMxH=CLDAR1
    PG2IOCONLbits.DBDAT = 0b00; // Data for PWMxH/PWMxL Pins if Debug Mode Event is Active: PWMxL=DBDAT0, PWMxH=DBDAR1

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG2IOCONHbits.CAPSRC = 0b000;  // Time Base Capture Source Selection: No hardware source selected for time base capture ? software only
    PG2IOCONHbits.DTCMPSEL = 0; // Dead-Time Compensation Selection: Dead-time compensation is controlled by PCI Sync logic
    PG2IOCONHbits.PMOD = 0b01; // PWM Generator Output Mode Selection: PWM Generator outputs operate in Independent mode
    PG2IOCONHbits.PENH = 0; // PWMxH Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxH output pin
    PG2IOCONHbits.PENL = 0; // PWMxL Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxL output pin
    PG2IOCONHbits.POLH = 0; // PWMxH Output Port Enable: Output pin is active-high
    PG2IOCONHbits.POLL = 0; // PWMxL Output Port Enable: Output pin is active-high
    
    // PWM GENERATOR 1 STATUS REGISTER
    PG2STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR 1 EVENT REGISTER LOW 
    PG2EVTLbits.ADTR1PS     = 0b00000;      // ADC Trigger 1 Postscaler Selection = 1:1 -> to be used for starting the ramp for slope compensation
    PG2EVTLbits.ADTR1EN3    = 0b0;          // PG2TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG2EVTLbits.ADTR1EN2    = 0b0;          // PG2TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG2EVTLbits.ADTR1EN1    = 0b1;          // PG2TRIGA  Compare Event is enabled as trigger source for ADC Trigger 1
    PG2EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG2STAT<4>) manually
    PG2EVTLbits.PGTRGSEL    = 0b000;        // PWM Generator Trigger Output is not used, leave it as default: EOC event is the PWM Generator trigger
    
    // PWM GENERATOR 1 EVENT REGISTER HIGH
    PG2EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG2EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG2EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG2EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG2EVTHbits.IEVTSEL     = 0b11;         // Time base interrupts are disabled
    PG2EVTHbits.ADTR2EN3    = 0b0;          // PG2TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG2EVTHbits.ADTR2EN2    = 0b0;          // PG2TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG2EVTHbits.ADTR2EN1    = 0b0;          // PG2TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG2EVTHbits.ADTR1OFS    = 0b00001;      // ADC Trigger 1 offset = Offset by 1 trigger event
    
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
    
    // Reset further PCI control registers
    PG2FPCIH        = 0x0000;          // PWM GENERATOR F PCI REGISTER HIGH
    PG2FPCIL        = 0x0000;          // PWM GENERATOR F PCI REGISTER LOW
    PG2FFPCIH       = 0x0000;          // PWM GENERATOR FF PCI REGISTER HIGH
    PG2FFPCIL       = 0x0000;          // PWM GENERATOR FF PCI REGISTER LOW
    PG2SPCIH        = 0x0000;          // PWM GENERATOR S PCI REGISTER HIGH
    PG2SPCIL        = 0x0000;          // PWM GENERATOR S PCI REGISTER LOW
    
    // PWM GENERATOR 2 LEADING-EDGE BLANKING REGISTER HIGH 
    PG2LEBHbits.PWMPCI      = 0b000;        // PWM Generator #1 output is made available to PCI logic
    PG2LEBHbits.PHR         = 0b1;          // Rising edge of PWM2H will trigger the LEB duration counter
    PG2LEBHbits.PHF         = 0b0;          // LEB ignores the falling edge of PWM2H
    PG2LEBHbits.PLR         = 0b0;          // LEB ignores the rising edge of PWM2L
    PG2LEBHbits.PLF         = 0b0;          // LEB ignores the falling edge of PWM2L
    
    // PWM GENERATOR 1 LEADING-EDGE BLANKING REGISTER LOW 
    PG2LEBL                 = 52;           // ToDo: This value may need further adjustment
    
    // PG2PHASE: PWM GENERATOR 1 PHASE REGISTER
    PG2PHASE    = 0;
    
    // PG2DC: PWM GENERATOR 1 DUTY CYCLE REGISTER
    PG2DC       = 800;      // 80%
    
    // PG2DCA: PWM GENERATOR 1 DUTY CYCLE ADJUSTMENT REGISTER
    PG2DCA      =  0x0000;      
    
    // PG2PER: PWM GENERATOR 1 PERIOD REGISTER        
    PG2PER      = 1000;     // 500 kHz

    // PG2TRIGA: PWM GENERATOR 1 TRIGGER A REGISTER
    PG2TRIGA    = 100;  // ToDo: Check this value on oscilloscope
    
    // PG2TRIGB: PWM GENERATOR 1 TRIGGER B REGISTER       
    PG2TRIGB    = 150;  // ToDo: Check this value on oscilloscope
    
    // PG2TRIGC: PWM GENERATOR 1 TRIGGER C REGISTER        
    PG2TRIGC    = 800;  // ToDo: Check this value on oscilloscope
    
    // PG2DTL: PWM GENERATOR 1 DEAD-TIME REGISTER LOW        
    PG2DTL      = 40;
    
    // PG2DTH: PWM GENERATOR 1 DEAD-TIME REGISTER HIGH
    PG2DTH      = 20;
            
//  PG2CAP      = 0x0000;   // Read only register
    
    return(1);
}        

volatile uint16_t launch_boost_pwm(void) {
    
    Nop();
    Nop();
    Nop();
    
    PG2CONLbits.ON = 1; // PWM Generator #2 Enable: PWM Generator is enabled
    PG2STATbits.UPDREQ = 1; // Update all PWM registers

    PG2IOCONHbits.PENH = 0; // PWMxH Output Port Enable: PWM generator controls the PWMxH output pin
    PG2IOCONHbits.PENL = 1; // PWMxL Output Port Enable: PWM generator controls the PWMxL output pin
    PG2IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG2IOCONLbits.OVRENL = 0;  // User Override Enable for PWMxL Pin: OVRDAT0 provides data for output on the PWMxL pin

    return(1);
}

// This PWM is used only to generate synchronized ADC Trigger 1 for the buck converter
volatile uint16_t init_buck_trig_pwm(void) {

    // PWM GENERATOR 3 CONTROL REGISTERS
    PG3CONLbits.ON = 0; // PWM Generator #3 Enable: PWM Generator is not enabled
    PG3CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG3CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator 1
    PG3CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG3CONLbits.MODSEL = 0b000; // PWM Mode Selection: Independent Edge PWM mode
    
    PG3CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG3CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG3CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG3CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG3CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG3CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG3CONHbits.SOCS = 0b0000; // Start-of-Cycle Selection: Local EOC PWM Generator is self-triggered

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER LOW
    PG3IOCONL = 0x0000;
    
    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG3IOCONHbits.CAPSRC = 0b000;  // Time Base Capture Source Selection: No hardware source selected for time base capture ? software only
    PG3IOCONHbits.DTCMPSEL = 0; // Dead-Time Compensation Selection: Dead-time compensation is controlled by PCI Sync logic
    PG3IOCONHbits.PMOD = 0b00; // PWM Generator Output Mode Selection: PWM Generator outputs operate in Complementary mode
    PG3IOCONHbits.PENH = 0; // PWMxH Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxH output pin
    PG3IOCONHbits.PENL = 0; // PWMxL Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxL output pin
    PG3IOCONHbits.POLH = 0; // PWMxH Output Port Enable: Output pin is active-high
    PG3IOCONHbits.POLL = 0; // PWMxL Output Port Enable: Output pin is active-high
    
    // PWM GENERATOR 1 STATUS REGISTER
    PG3STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR 1 EVENT REGISTER LOW 
    PG3EVTLbits.ADTR1PS     = 0b00001;      // ADC Trigger 1 Postscaler Selection = 1:2
    PG3EVTLbits.ADTR1EN3    = 0b0;          // PG1TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.ADTR1EN2    = 0b0;          // PG1TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.ADTR1EN1    = 0b1;          // PG1TRIGA  Compare Event is enabled as trigger source for ADC Trigger 1
    PG3EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG1STAT<4>) manually
    PG3EVTLbits.PGTRGSEL    = 0b000;        // PWM Generator Trigger Output is not used, leave it as default: EOC event is the PWM Generator trigger
    
    // PWM GENERATOR 1 EVENT REGISTER HIGH
    PG3EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG3EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG3EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG3EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG3EVTHbits.IEVTSEL     = 0b10;         // Interrupt Event Selection: Interrupts CPU at ADC Trigger 1 event; ToDo: This is just for debugging
    PG3EVTHbits.ADTR2EN3    = 0b0;          // PG1TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR2EN2    = 0b0;          // PG1TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR2EN1    = 0b0;          // PG1TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR1OFS    = 0b00000;      // ADC Trigger 1 offset = No offset (auxiliary PG for boost should have an offset with respect to this)
    
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
    
        
    // PG1PHASE: PWM GENERATOR 1 PHASE REGISTER
    PG3PHASE    = 0;
    
    // PG1DC: PWM GENERATOR 1 DUTY CYCLE REGISTER
    PG3DC       = 800;      // 80%
    
    // PG1DCA: PWM GENERATOR 1 DUTY CYCLE ADJUSTMENT REGISTER
    PG3DCA      =  0x0000;      
    
    // PG1PER: PWM GENERATOR 1 PERIOD REGISTER        
    PG3PER      = 1000;     // 500 kHz

    // PG1TRIGA: PWM GENERATOR 1 TRIGGER A REGISTER
    PG3TRIGA    = 100;  // ToDo: Check this value on oscilloscope
    
    // PG1TRIGB: PWM GENERATOR 1 TRIGGER B REGISTER       
    PG3TRIGB    = 0;  
    
    // PG1TRIGC: PWM GENERATOR 1 TRIGGER C REGISTER        
    PG3TRIGC    = 0;  
    
    // PG1DTL: PWM GENERATOR 1 DEAD-TIME REGISTER LOW        
    PG3DTL      = 40;
    
    // PG1DTH: PWM GENERATOR 1 DEAD-TIME REGISTER HIGH
    PG3DTH      = 20;
            
//  PG3CAP      = 0x0000;   // Read only register
   
    return(1);
}

// This PWM is used only to generate synchronized ADC Trigger 1 for the boost converter
volatile uint16_t init_boost_trig_pwm(void) {

    // PWM GENERATOR 4 CONTROL REGISTERS
    PG4CONLbits.ON = 0; // PWM Generator #4 Enable: PWM Generator is not enabled
    PG4CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG4CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator 4
    PG4CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG4CONLbits.MODSEL = 0b000; // PWM Mode Selection: Independent Edge PWM mode
    
    PG4CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG4CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG4CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG4CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG4CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG4CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG4CONHbits.SOCS = 0b0000; // Start-of-Cycle Selection: Local EOC PWM Generator is self-triggered; ToDo: Maybe it's better to be triggered by the boost PG?

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
    PG4EVTLbits.PGTRGSEL    = 0b000;        // PWM Generator Trigger Output is not used, leave it as default: EOC event is the PWM Generator trigger
    
    // PWM GENERATOR x EVENT REGISTER HIGH
    PG4EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG4EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG4EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG4EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG4EVTHbits.IEVTSEL     = 0b10;         // Interrupt Event Selection: Interrupts CPU at ADC Trigger 1 event; ToDo: This is just for debugging
    PG4EVTHbits.ADTR2EN3    = 0b0;          // PG1TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR2EN2    = 0b0;          // PG1TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR2EN1    = 0b0;          // PG1TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR1OFS    = 0b00001;      // ADC Trigger 1 offset = 1 PWM cycle 
    
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
    
        
    // PG1PHASE: PWM GENERATOR 1 PHASE REGISTER
    PG4PHASE    = 0;
    
    // PG1DC: PWM GENERATOR 1 DUTY CYCLE REGISTER
    PG4DC       = 800;      // 80%
    
    // PG1DCA: PWM GENERATOR 1 DUTY CYCLE ADJUSTMENT REGISTER
    PG4DCA      =  0x0000;      
    
    // PG1PER: PWM GENERATOR 1 PERIOD REGISTER        
    PG4PER      = 1000;     // 500 kHz

    // PG1TRIGA: PWM GENERATOR 1 TRIGGER A REGISTER
    PG4TRIGA    = 100;  // ToDo: Check this value on oscilloscope
    
    // PG1TRIGB: PWM GENERATOR 1 TRIGGER B REGISTER       
    PG4TRIGB    = 0;  
    
    // PG1TRIGC: PWM GENERATOR 1 TRIGGER C REGISTER        
    PG4TRIGC    = 0;  
    
    // PG1DTL: PWM GENERATOR 1 DEAD-TIME REGISTER LOW        
    PG4DTL      = 40;
    
    // PG1DTH: PWM GENERATOR 1 DEAD-TIME REGISTER HIGH
    PG4DTH      = 20;
            
//  PG4CAP      = 0x0000;   // Read only register
   
    return(1);
}