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

volatile uint16_t init_buck_pwr_control(void) {
    
    init_buck_pwm();    // Set up buck converter PWM
    init_buck_acmp();   // Set up buck converter peak current comparator/DAC
    init_buck_adc();    // Set up buck converter ADC (voltage feedback only)
    
    return(1);
}

volatile uint16_t launch_buck_pwr_control(void) {

//    volatile uint16_t _i = 0;

    // Run enable-sequence of all peripherals used by this power controller
    launch_buck_acmp();
    launch_buck_pwm();
    
    return(1);
}

volatile uint16_t exec_buck_pwr_control(void) {
    
    
    return(1);
}