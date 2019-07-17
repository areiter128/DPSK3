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
#include "npnz16b.h"


volatile bool adc_active = false;
volatile BUCK_SOFT_START_t buck_soft_start;

volatile uint16_t init_buck_pwr_control(void) {
    
    init_buck_trig_pwm();   // Set up auxiliary PWM for buck converter
    init_buck_pwm();        // Set up buck converter PWM
    init_buck_acmp();       // Set up buck converter peak current comparator/DAC
    init_buck_adc();        // Set up buck converter ADC (voltage feedback only)
    
    buck_soft_start.counter = 0;            // Reset Soft-Start Counter
    buck_soft_start.phase = BUCK_SS_INIT;   // Reset Soft-Start Phase to Initialization
    buck_soft_start.pwr_good_delay = 999;   // Soft-Start Power-On Delay = 100 ms
    buck_soft_start.ramp_period = 499;      // Soft-Start Ramp Period = 500 ms
    buck_soft_start.pwr_good_delay = 1999;  // Soft-Start Power Good Delay = 200 ms
    buck_soft_start.reference = 2047;       // Soft-Start Target Reference = 3.3V
    
    c2p2z_buck_Init();
    
    c2p2z_buck.ADCTriggerOffset = 80;
    c2p2z_buck.ptrADCTriggerRegister = &PG3TRIGA;
    c2p2z_buck.InputOffset = 0;
    c2p2z_buck.ptrControlReference = &data.buck_vref;
    c2p2z_buck.ptrSource = &ADCBUF13;
    c2p2z_buck.ptrTarget = &DAC1DATH;
    c2p2z_buck.MaxOutput = 3600;
    c2p2z_buck.MinOutput = 10;
    c2p2z_buck.status.flag.enable = 0;
    
    data.buck_vref = 0;
    
    return(1);
}

volatile uint16_t launch_buck_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    launch_adc();           // Start ADC Module
    launch_buck_acmp();     // Start analog comparator/DAC module
    launch_buck_trig_pwm(); // Start auxiliary PWM 
    launch_buck_pwm();      // Start PWM
    
    while (!adc_active); 
    c2p2z_buck.status.flag.enable = 1; // Start the control loop for buck
    
    return(1);
}

volatile uint16_t exec_buck_pwr_control(void) {
    
    
    switch (buck_soft_start.phase) {
        
        case 0: // basic PWM, ADC, CMP, DAC configuration
            init_buck_pwr_control();    // Initialize all peripherals and data structures of the buck controller
            buck_soft_start.phase = 1;
            break;

        case 1: // Enabling PWM, ADC, CMP, DAC
            launch_buck_pwr_control();  // Start Buck Power Controller
            buck_soft_start.phase = 2;
            break;

        case 2:
            if (data.buck_vref < buck_soft_start.reference) 
            { data.buck_vref++; }
            else
            {  buck_soft_start.phase = 3; }
            
        case 3: // Soft start is complete, system is running
            Nop();
            break;

        default: // If something is going wrong, reset entire PWR controller
            buck_soft_start.phase = 0;
            break;
            
    }
        
    
    return(1);
}

void __attribute__((__interrupt__, auto_psv)) _ADCAN13Interrupt(void)
{
    volatile uint16_t dummy=0;
    
    adc_active = true;
    
//    dummy = ADCBUF13;

    data.vout_buck = ADCBUF13;
    c2p2z_buck_Update(&c2p2z_buck);

    _ADCAN13IF = 0;  // Clear the ADCANx interrupt flag 
    
}

