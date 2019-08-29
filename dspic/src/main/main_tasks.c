//=======================================================================================================
// Copyright(c) 2018 Microchip Technology Inc. and its subsidiaries.
// Subject to your compliance with these terms, you may use Microchip software and any derivatives
// exclusively with Microchip products. It is your responsibility to comply with third party license
// terms applicable to your use of third-party software (including open source software) that may
// accompany Microchip software.
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
// APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND
// FITNESS FOR A PARTICULAR PURPOSE.
// IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
// LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
// MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT
// ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT
// EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//=======================================================================================================

//=======================================================================================================
// @file main_tasks.c
//
// @brief contains the main tasks that are called by the scheduler
//        two different timings priorities are available:
//          1. 100�s and 1ms Tasks called from the scheduler interrupt
//              the jitter that you will have in the 100�s realtime tasks called by the interrupt depends
//              on other interrupts that have a higher interrupt priority
//              the jitter that you will have in the 1ms realtime tasks called by the interrupt depends
//              on other interrupts that have a higher interrupt priority amd by the duration of the
//              100�s realtime task
//          2. 100�s, 1ms, 10ms, 100ms, 1s Tasks called from the main loop
//              these tasks are for soft realtime and not for hard realtime
//              so in average they are called with the required timing but the jitter can be very huge,
//              depending on the calls before.
//              use this for your non-timing critical application state machines
//
// @version v1.0
// @date 2019-08-29
// @author M52409
//
//=======================================================================================================

#include <stdint.h>
#include "init/init_gpio.h"
#include "driver/power_controllers/drv_power_controllers.h"
#include "device/dev_button.h"
#include "app/app_fault_handling.h"
#include "app/app_proto24.h"
#include "app/app_display.h"
#include "app/app_logger.h"
#include "misc/global.h"


//=======================================================================================================
//
//                          put your application specific code in the following functions:
//                              choose wisely between real-time and non-realtime!
//
//  Interrupt Realtime Functions:
//  Tasks_Realtime_100us:   is called by the 100�s interrupt    - for time critical low jitter stuff
//  Tasks_Realtime_1ms  :   is called by the interrupt every ms - for time critical low jitter stuff
//
//
//  Mainloop Non-Realtime Functions:
//  Tasks_100us         :   function is called by the main loop in average every 100�s
//  Tasks_1ms           :   function is called by the main loop in average every 1ms
//  Tasks_10ms          :   function is called by the main loop in average every 10ms
//  Tasks_100ms         :   function is called by the main loop in average every 100ms
//  Tasks_1s            :   function is called by the main loop in average every second
//
//  @note there could be some jitter here because it is not called directly by a timer interrupt
//        the timing in average is exact (keep in mind: in average), the jitter depends on the
//        called functions before
//=======================================================================================================


volatile uint16_t time_counter_logger = 0;      // for the timing of the logger

//=======================================================================================================
//  @brief  Tasks_Realtime_100us gets called directly from the timer interrupt every 100�s
//  @note   keep this routine as short as possible
//=======================================================================================================
void Tasks_Realtime_100us(void)
{
    
}

//=======================================================================================================
//  @brief  Tasks_Realtime_1ms gets called directly from the timer interrupt every millisecond
//  @note   keep this routine as short as possible
//=======================================================================================================
void Tasks_Realtime_1ms(void)
{
    
}

//=======================================================================================================
//  @brief  Tasks_100us gets called every 100�s, put your things in it that need to be called that often
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//=======================================================================================================
void Tasks_100us(void)
{
    DBGPIN_1_TOGGLE; // Toggle DEBUG-PIN

    Drv_PowerControllers_Task_100us();
    //exec_boos_pwr_control();
}

//=======================================================================================================
//  @brief  Tasks_1ms gets called every millisecond, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//=======================================================================================================
void Tasks_1ms(void)
{
    App_Fault_Handling_Task_1ms();
    App_Proto24_Task_1ms();
    if(App_Proto24_IsNewDataAvailable())
    {
        App_Proto24_GetData(&global_proto24data);
        Global_UpdateProto24Data();
    }
}

//=======================================================================================================
//  @brief  Tasks_10ms gets called every 10ms, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//=======================================================================================================
void Tasks_10ms(void)
{
    Dev_Button_Task_10ms();
}

//=======================================================================================================
//  @brief  Tasks_100ms gets called every 100 ms, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//=======================================================================================================
void Tasks_100ms(void)
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

//=======================================================================================================
//  @brief  Tasks_1s gets called every second, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//=======================================================================================================
void Tasks_1s(void)
{
    // put your application specific code here that needs to be called every second
}

//=======================================================================================================
//  @brief  Tasks_Background gets called all the time when no other of the above tasks are being called
//  @note   call this function when you want to implement your own timing or get code called as often
//          as possible. You can also put your timing variables into Tasks_Realtime_100us or
//          Tasks_Realtime_1ms. This way you get accurate timing variables that you can use here.
//=======================================================================================================
void Tasks_Background(void)
{
    // put your application specific code here that needs to be called in the background
    // your application needs to take care of it's timing.
}