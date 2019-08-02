//======================================================================================================================
// Copyright(c) 2018 Microchip Technology Inc. and its subsidiaries.
// Subject to your compliance with these terms, you may use Microchip software and any derivatives exclusively with
// Microchip products. It is your responsibility to comply with third party license terms applicable to your use of
// third-party software (including open source software) that may accompany Microchip software.
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO
// THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
// PURPOSE.
// IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE,
// COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED
// OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY
// ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE
// PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//======================================================================================================================

//======================================================================================================================
// @file main.c
//
// @brief contains the main function for the Digital Power Starter Kit Version 3 (aka DPSK3)
//
// @author M91406
// @author M91281
// @author M52409
//
// @date July 8, 2019, 1:52 PM
//======================================================================================================================

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "device/dev_lcd.h"
#include "device/dev_button.h"
#include "device/dev_uart1.h"
#include "driver/drv_buck_power_controller.h"
#include "app/app_display.h"
#include "app/app_proto24.h"
#include "app/app_logger.h"
#include "app/app_fault_handling.h"

#include "init/init_fosc.h"
#include "init/init_gpio.h"
#include "init/init_pwm.h"
#include "init/init_acmp.h"
#include "init/init_adc.h"

#include "main/main_scheduler.h"

#ifdef TEST_ENABLED
#include "app/app_test.h"
#endif  // TEST_ENABLED
#include "misc/delay.h"
#include "misc/global.h"
#include "misc/dummy_compensator.h"
//#include "main.h"

volatile uint16_t timer1_timeout_counter = 0;
volatile uint32_t timer1_isdead_counter = 0;

int main(void)
{
    //TODO: general system setup that is not part of some other component of the software like oscillators etc.
    // should be moved in a drv_system.c file with a Drv_System_Init() function
    // all other low level peripheral initializations should be moved to the Init-Routine of the corresponding software module(e.g. power controller)
    init_fosc();        // Set up system oscillator for 100 MIPS operation
    init_aclk();        // Set up Auxiliary PLL for 500 MHz (source clock to PWM module)
    init_gpio();        // Initialize common device GPIOs
    
    // Basic setup of common power controller peripheral modules
    init_pwm_module();  // Set up PWM module (basic module configuration)
    init_acmp_module(); // Set up analog comparator/DAC module
    init_adc_module();  // Set up Analog-To-Digital converter module
    init_vin_adc();     // Initialize ADC Channel to measure input voltage
    
    Dev_Lcd_Init();
    Dev_Button_Init();
    App_Logger_Init();
    App_Fault_Handling_Init();
    App_Proto24_Init();
    App_Proto24_GetData(&global_proto24data);
    Main_Scheduler_Init();
    //TODO: is that delay for sending the SYS_RESET information after about one second to make sure that the pic24 is also ready?
    //TODO: this could also be integrated in the App_Proto24_Task_10ms function where we have a more accurate timing
    __delay_ms(200); // due to heavy workload on ISR, delays take ~3.7 times longer than normal

    Drv_BuckPowerController_Init();                 //Init the power controller of the buck converter
    Drv_BuckPowerController_SetOutputVoltageReference(3.3);  //Set Buck Controller Output to 3.3 Volt

//    init_boost_pwr_control();   // Initialize all peripherals and data structures of the boost controller
//    launch_boost_pwr_control(); // Start Buck Power Controller

    // the PROTO_SYS_RESET message was eliminated because the pic24 does reset itself too which it should not do
    //App_Proto24_Send(PROTO_SYS_RESET);

    Global_UpdateBoardData();
    global_data.pic24_packet_counter = 0;

    //Dev_Lcd_WriteStringXY(0,0,"MICROCHIP  dsPIC");
    //Dev_Lcd_WriteStringXY(0,1,"  33CK256MP505  ");
    
#ifdef TEST_ENABLED
    App_Test();
#else
    Main_Scheduler_RunForever();
#endif  // TEST_ENABLED
    return (0);
}

