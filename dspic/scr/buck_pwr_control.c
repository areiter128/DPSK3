/*
 * File:   pwr_control.c
 * Author: M91406
 *
 * Created on July 9, 2019, 1:10 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

volatile BUCK_SOFT_START_t buck_soft_start;

volatile uint16_t init_buck_pwr_control(void) {
    
    init_buck_pwm();    // Set up buck converter PWM
    init_buck_acmp();   // Set up buck converter peak current comparator/DAC
    init_buck_adc();    // Set up buck converter ADC (voltage feedback only)
    
    buck_soft_start.counter = 0;            // Reset Soft-Start Counter
    buck_soft_start.phase = BUCK_SS_INIT;   // Reset Soft-Start Phase to Initialization
    buck_soft_start.pwr_good_delay = 999;   // Soft-Start Power-On Delay = 100 ms
    buck_soft_start.ramp_period = 499;      // Soft-Start Ramp Period = 500 ms
    buck_soft_start.pwr_good_delay = 1999;  // Soft-Start Power Good Delay = 200 ms
    buck_soft_start.reference = 2047;       // Soft-Start Target Reference = 3.3V
        
    return(1);
}

volatile uint16_t launch_buck_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    launch_adc();       // Start ADC Module
    launch_buck_acmp(); // Start analog comparator/DAC module
    launch_buck_pwm();  // Start PWM
    
    while(PG1DC < 250)
    {
        PG1DC++;
        PG1STATbits.UPDREQ = 1;
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
    }
    
    return(1);
}

volatile uint16_t exec_buck_pwr_control(void) {
    
    
    return(1);
}

void __attribute__((__interrupt__, auto_psv)) _ADCAN13Interrupt(void)
{
    volatile uint16_t dummy=0;
    
    dummy = ADCBUF13;

    Nop();
    Nop();
    Nop();

    _ADCAN13IF = 0;  // Clear the ADCANx interrupt flag 
    
}

