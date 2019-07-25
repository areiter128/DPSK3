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
#ifdef TEST_ENABLED
#include "app/app_test.h"
#endif  // TEST_ENABLED
#include "misc/delay.h"
#include "misc/global.h"
#include "misc/dummy_compensator.h"
#include "main.h"

volatile MY_DATA_POINTS_t data;

volatile uint16_t counter_100us_leader = 0;     // local counter for 100us interrupt - leader
volatile uint16_t counter_100us_follower = 0;   // local counter for 100us interrupt - follower
volatile uint16_t tasks_1ms_counter = 0;        // local counter for 1ms tasks
volatile uint16_t tasks_10ms_counter = 0;       // local counter for 10ms tasks
volatile uint16_t tasks_100ms_counter = 0;      // local counter for 100ms tasks
volatile uint16_t time_counter_logger = 0;      // for the timing of the logger
volatile uint16_t timer1_timeout_counter = 0;
volatile uint32_t timer1_isdead_counter = 0;

inline void Tasks_MainTaskLoop(void);
inline void Tasks_100us(void);
inline void Tasks_1ms(void);
inline void Tasks_10ms(void);
inline void Tasks_100ms(void);

int main(void)
{
    init_fosc();        // Set up system oscillator for 100 MIPS operation
    init_aclk();        // Set up Auxiliary PLL for 500 MHz (source clock to PWM module)
    init_timer1();      // Set up Timer1 as main scheduler time base
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
    
    counter_100us_follower = 0;
    counter_100us_leader = 0;
    
#ifdef TEST_ENABLED
    App_Test();
#else
    Tasks_MainTaskLoop();
#endif  // TEST_ENABLED
    return (0);
}

//======================================================================================================================
//  @brief  Timer 1 interrupt routine for generating the main highres timing for all the tasks
//  @note   with this simple implementation we do not lose any tick from timer 1, even when the tasks in the main loop
//  @note   take longer than 100 µs
//======================================================================================================================
void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void)
{
    counter_100us_leader++;     //increment our counter, no tick gets lost this way
    _T1IF = 0;                  //clear Timer1 interrupt flag
}

inline void Tasks_MainTaskLoop(void)
{
    //TODO: should we implement a Watchdog that gets triggered in one of the Task-Routines?
    while (1)   //100µs scheduler
    {
        if (counter_100us_leader != counter_100us_follower)
        {
            counter_100us_follower++;
            Tasks_100us();
        }
    }
}

//======================================================================================================================
//  @brief  the function Tasks_100us gets called every 100µs, put your things in it that need to be called that often
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//======================================================================================================================
inline void Tasks_100us(void)
{
    DBGPIN_1_TOGGLE; // Toggle DEBUG-PIN

    Drv_BuckPowerController_Task_100us();
    //exec_boos_pwr_control();

    if (tasks_1ms_counter++ >= 10)
    {
        tasks_1ms_counter = 0; //reset toggle counter
        Tasks_1ms();
    }
}

//======================================================================================================================
//  @brief  the function Tasks_1ms gets called every millisecond, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//======================================================================================================================
inline void Tasks_1ms(void)
{
    App_Fault_Handling_Task_1ms();
    App_Proto24_Task_1ms();
    if(App_Proto24_IsNewDataAvailable())
    {
        App_Proto24_GetData(&global_proto24data);
        Global_UpdateProto24Data();
    }
    if (tasks_10ms_counter++ >= 10)
    {
        tasks_10ms_counter = 0; //reset toggle counter
        Tasks_10ms();
    }
}

//======================================================================================================================
//  @brief  the function Tasks_10ms gets called every 10ms, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//======================================================================================================================
inline void Tasks_10ms(void)
{
    Dev_Button_Task_10ms();
    
    if (tasks_100ms_counter++ >= 10)
    {
        tasks_100ms_counter = 0; //reset toggle counter
        Tasks_100ms();
    }
}

//======================================================================================================================
//  @brief  the function Tasks_100ms gets called every 100 ms, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//======================================================================================================================
inline void Tasks_100ms(void)
{
    DBGLED_TOGGLE;              // Toggle debug LED
    App_Display_Task_100ms();   // calling the display application that contains the main state machine
    Global_UpdateBoardData();
    
    time_counter_logger++;
    if (time_counter_logger == 8)
        App_Logger_LogData();
    else if (time_counter_logger == 16)
    {   
        App_Logger_LogProto24();
        time_counter_logger = 0;
    }
}

