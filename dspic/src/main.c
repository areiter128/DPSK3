/*
 * File:   main.c
 * Author: M91406
 *
 * Created on July 8, 2019, 1:52 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

extern volatile uint16_t VOUTbuck, VOUTboost;
volatile uint16_t vbuck, vboost;

volatile uint16_t tgl_cnt = 0;  // local counter of LED toggle loops
#define TGL_INTERVAL    2999     // LED toggle interval of (2999 + 1) x 100usec = 100ms
#define TMR1_TIMEOUT    30000   // Timeout protection for Timer1 interrupt flag bit

int main(void) {

    volatile uint16_t timeout = 0;

    init_fosc();
    init_timer1();
    init_gpio();
    
    // Basic setup of common power controller peripheral modules
    init_aclk();        // Set up Auxiliary PLL for 500 MHz (source clock to PWM module)
    init_pwm_module();  // Set up PWM module (basic module configuration)
    init_acmp_module(); // Set up analog comparator/DAC module
    init_adc_module();  // Set up Analog-To-Digital converter module
    init_vin_adc();     // Initialize ADC Channel to measure input voltage
    
    // Initialize peripheral modules of individual power controllers
    init_buck_pwr_control();    // Initialize all peripherals and data structures of the buck controller
    init_boost_pwr_control();   // Initialize all peripherals and data structures of the boost controller

    launch_buck_pwr_control();  // Start Buck Power Controller
    launch_boost_pwr_control(); // Start Buck Power Controller
    
    while (1) {

        // wait for timer1 to overrun
        while ((!_T1IF) && (timeout++ < TMR1_TIMEOUT));
        timeout = 0;    // Reset timeout counter
        _T1IF = 0; // reset Timer1 interrupt flag bit
        DBGPIN_1_TOGGLE; // Toggle DEBUG-PIN

        if (tgl_cnt++ > TGL_INTERVAL) // Count 100usec loops until LED toggle interval is exceeded
        {
            DBGLED_TOGGLE;
            tgl_cnt = 0;
            
            vbuck = VOUTbuck;
            vboost = VOUTboost;
        } // Toggle LED and reset toggle counter

    }


    return (0);
}
