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
// @file main_scheduler.c
//
// @brief contains the main scheduler that calls all the tasks that need to be called regularly
//        two different timings priorities are available:
//          1. 100탎 and 1ms Tasks called from the scheduler interrupt
//              the jitter that you will have in the 100탎 realtime tasks called by the interrupt depends
//              on other interrupts that have a higher interrupt priority
//              the jitter that you will have in the 1ms realtime tasks called by the interrupt depends
//              on other interrupts that have a higher interrupt priority amd by the duration of the
//              100탎 realtime task
//          2. 100탎, 1ms, 10ms, 100ms, 1s Tasks called from the main loop
//              these tasks are for soft realtime and not for hard realtime
//              so in average they are called with the required timing but the jitter can be very huge,
//              depending on the calls before.
//              use this for your non-timing critical application state machines
//
// @version v1.0
// @date 2019-08-02
// @author M52409
//
//=======================================================================================================

#include <stdint.h>
#include "init/init_gpio.h"
#include "driver/drv_buck_power_controller.h"
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
//  Tasks_Realtime_100us:   is called by the 100탎 interrupt    - for time critical low jitter stuff
//  Tasks_Realtime_1ms  :   is called by the interrupt every ms - for time critical low jitter stuff
//
//
//  Mainloop Non-Realtime Functions:
//  Tasks_100us         :   function is called by the main loop in average every 100탎
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
//  @brief  Tasks_Realtime_100us gets called directly from the timer interrupt every 100탎
//  @note   keep this routine as short as possible
//=======================================================================================================
static inline void Tasks_Realtime_100us(void)
{
    
}

//=======================================================================================================
//  @brief  Tasks_Realtime_1ms gets called directly from the timer interrupt every millisecond
//  @note   keep this routine as short as possible
//=======================================================================================================
static inline void Tasks_Realtime_1ms(void)
{
    
}

//=======================================================================================================
//  @brief  Tasks_100us gets called every 100탎, put your things in it that need to be called that often
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//=======================================================================================================
static inline void Tasks_100us(void)
{
    DBGPIN_1_TOGGLE; // Toggle DEBUG-PIN

    Drv_BuckPowerController_Task_100us();
    //exec_boos_pwr_control();
}

//=======================================================================================================
//  @brief  Tasks_1ms gets called every millisecond, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//=======================================================================================================
static inline void Tasks_1ms(void)
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
static inline void Tasks_10ms(void)
{
    Dev_Button_Task_10ms();
}

//=======================================================================================================
//  @brief  Tasks_100ms gets called every 100 ms, put your things in it that need to be called regularly
//  @note   there could be some jitter here because it is not called directly by a timer interrupt
//=======================================================================================================
static inline void Tasks_100ms(void)
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
static inline void Tasks_1s(void)
{
    // put your application specific code here that needs to be called every second
}





//=======================================================================================================
//=======================================================================================================
//
//                the source code after this point usually should not be modified,
//
//                   only in certain cases when you don't know a different way
//
//            instead use the task functions above to put in your application specific code
//
//=======================================================================================================
//=======================================================================================================


static volatile uint16_t   scheduler_interrupt_leader_100us = 0;
static volatile uint16_t   scheduler_interrupt_follower_100us = 0;
static volatile uint8_t    scheduler_interrupt_realtime_counter_1ms = 0;


//=======================================================================================================
//  @brief  Initializes Scheduler
//  @note   call this function in your main routine before calling the RunForever function
//=======================================================================================================
void Main_Scheduler_Init(void)
{
    T1CONbits.TON = 0;      // Timer1 On: Stops 16-bit Timer1 during configuration
    T1CONbits.TSIDL = 0;    // Timer1 Stop in Idle Mode: Continues module operation in Idle mode
    T1CONbits.TMWDIS = 0;   // Asynchronous Timer1 Write Disable: Back-to-back writes are enabled in Asynchronous mode
    T1CONbits.TMWIP = 0;    // Asynchronous Timer1 Write in Progress: Write to the timer in Asynchronous mode is complete
    T1CONbits.PRWIP = 0;    // Asynchronous Period Write in Progress: Write to the Period register in Asynchronous mode is complete
    T1CONbits.TECS = 0b11;  // Timer1 Extended Clock Select: FRC clock
    T1CONbits.TGATE = 0;    // Timer1 Gated Time Accumulation Enable: Gated time accumulation is disabled when TCS = 0
    T1CONbits.TCKPS = 0;    // Timer1 Input Clock Prescale Select: 1:1
    T1CONbits.TSYNC = 0;    // Timer1 External Clock Input Synchronization Select: Does not synchronize the External Clock input
    T1CONbits.TCS = 0;      // Timer1 Clock Source Select: Internal peripheral clock
    
    TMR1 = 0x00;            // Reset Timer Counter Register TMR to Zero; 
    PR1 = 9999;             // Period = 0.0001 s; Frequency = 100000000 Hz; PR 9999
    IPC0bits.T1IP = 1;      // Set interrupt priority to one (cpu is running on ip zero)
    IFS0bits.T1IF = 0;      // Reset interrupt flag bit
    IEC0bits.T1IE = 1;      // Enable Timer1 interrupt

    T1CONbits.TON = 1;      // Enable Timer1 now
}

//=======================================================================================================
//  @brief  Timer 1 interrupt routine for generating the 100탎 timing for the scheduler
//  @note   with this simple implementation we do not lose any tick from the timer, even when the tasks
//          in the main loop take longer than 100탎
//=======================================================================================================
void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void)
{
    scheduler_interrupt_leader_100us++; //increment our counter for the scheduler, no tick gets lost
    _T1IF = 0;                          //clear Timer1 interrupt flag
    Tasks_Realtime_100us();
    if (++scheduler_interrupt_realtime_counter_1ms >= 10)
    {
        Tasks_Realtime_1ms();
        scheduler_interrupt_realtime_counter_1ms = 0;
    }
}

//=======================================================================================================
//  @brief  Scheduler function for calling all the Tasks regularly ( 100us, 1ms, 10ms, 100ms, 1s )
//  @note   call this function as last function in main.c after calling the Init-function
//          please consider that the timing of the calls are dependent on the duration of the last call
//          the resulting jitter therefore depends on the timing of the calls before
//=======================================================================================================
void Main_Scheduler_RunForever(void)
{
    volatile static uint16_t scheduler_1ms_timer = 0;       // local counter for 1ms tasks
    volatile static uint16_t scheduler_10ms_timer = 0;      // local counter for 10ms tasks
    volatile static uint16_t scheduler_100ms_timer = 0;     // local counter for 100ms tasks
    volatile static uint16_t scheduler_1s_timer = 0;        // local counter for 1s tasks

    //TODO: should we implement a Watchdog that gets triggered in one of the Task-Routines?
    // do some initialization
    scheduler_interrupt_leader_100us = 0;   // reset directly before calling the Scheduler Loop
    scheduler_interrupt_follower_100us = 0; // reset directly before calling the Scheduler Loop
    
    while (1)                               // run that loop forever
    {
        if (scheduler_interrupt_follower_100us != scheduler_interrupt_leader_100us)
        {
            scheduler_interrupt_follower_100us++;
            Tasks_100us();                              //call 100탎 tasks
            if (scheduler_1ms_timer++ >= 10)
            {
                scheduler_1ms_timer = 0;                //reset 1 ms timer
                Tasks_1ms();                            //call 1 ms tasks
                if (scheduler_10ms_timer++ >= 10)
                {
                    scheduler_10ms_timer = 0;           //reset 10 ms timer
                    Tasks_10ms();                       //call 10 ms tasks
                    if (scheduler_100ms_timer++ >= 10)
                    {
                        scheduler_100ms_timer = 0;      //reset 100 ms timer
                        Tasks_100ms();                  //call 100 ms tasks
                        if (scheduler_1s_timer++ >= 10)
                        {
                            Tasks_1s();                  //call 1 s tasks
                        }                            
                    }
                }
            }
        }
    }
}
