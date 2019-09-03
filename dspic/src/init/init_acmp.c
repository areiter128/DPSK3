/*
 * File:   init_acmp.c
 * Author: M91406
 *
 * Created on July 9, 2019, 11:12 AM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "init_acmp.h"

//#define DACDATH_BUCK    0       // DAC value for the buck the slope starts from
//#define DACDATH_BOOST   0       // DAC value for the boost the slope starts from
//#define DACDATL_BUCK    205     // Set this to minimum in Slope mode
//#define DACDATL_BOOST   205     // Set this to minimum in Slope mode

//#define LEB_PER_COMP    50      // Leading edge period for the comparator when slope re-settles to its initial value

//#define TMOD_DURATION   75      // Transition Mode Duration
//#define SS_DURATION     85      // Time from Start of Transition Mode until Steady-State Filter is Enabled

//#define SLOPE_RATE      43      // Slope Ramp Rate Value

volatile uint16_t init_acmp_module(void) {

    
}

volatile uint16_t init_buck_acmp(void) {

    
}

volatile uint16_t launch_buck_acmp(void) {
    
    
}

volatile uint16_t init_boost_acmp(void) {
    

    // DACxCONL: DACx CONTROL LOW REGISTER
    DAC2CONLbits.DACEN = 0; // Individual DACx Module Enable: Disables DACx module during configuration
    DAC2CONLbits.IRQM = 0b00; // Interrupt Mode Selection: Interrupts are disabled
    DAC2CONLbits.CBE = 1; // Comparator Blank Enable: Enables the analog comparator output to be blanked (gated off) during the recovery transition following the completion of a slope operation
    DAC2CONLbits.DACOEN = 1; // DACx Output Buffer Enable: disabled for this module
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
    SLP2CONLbits.SLPSTOPB = 0b0010; // Slope Stop B Signal Selection: CMP2 Out
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

volatile uint16_t launch_boost_acmp(void) {
    
    DAC2CONLbits.DACEN = 1; // Individual DACx Module Enable: Enables DAC2 module 
    DACCTRL1Lbits.DACON = 1; // Common DAC Module Enable: Enables all enabled DAC modules
    
    return(1);
}

