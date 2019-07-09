/*
 * File:   pwr_control.c
 * Author: M91406
 *
 * Created on July 9, 2019, 1:10 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "boost_pwr_control.h"

volatile uint16_t init_boost_pwr_control(void) {

/*    ToDo: ENABLE THE BOOST CONVERTER CONTROL

    init_boost_pwm();    // Set up boost converter PWM
    init_boost_acmp();   // Set up boost converter peak current comparator/DAC
    init_boost_adc();    // Set up boost converter ADC (voltage feedback only)
*/    
    return(1);
}

volatile uint16_t launch_boost_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    
    // ToDo: ADD BOOST CONVERTER LAUNCH CODE
    
    return(1);
}

volatile uint16_t exec_boost_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    
    // ToDo: ADD BOOST CONVERTER EXECUTION CODE
    
    return(1);
}

