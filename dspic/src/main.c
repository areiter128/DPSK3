/*
 * File:   main.c
 * Author: M91406
 *
 * Created on July 8, 2019, 1:52 PM
 */


#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "device/dev_lcd.h"
#include "device/dev_button.h"
#include "device/dev_uart1.h"
#include "app/app_display.h"
#include "app/app_proto24.h"
#include "app/app_logger.h"
#include "misc/delay.h"
#include "misc/global.h"
#include "main.h"

#define TMR1_TIMEOUT    30000   // Timeout protection for Timer1 interrupt flag bit

volatile uint16_t counter_100us_leader = 0;     // local counter for 100us interrupt - leader
volatile uint16_t counter_100us_follower = 0;   // local counter for 100us interrupt - follower
volatile uint16_t tasks_1ms_counter = 0;        // local counter for 1ms tasks
volatile uint16_t tasks_10ms_counter = 0;       // local counter for 10ms tasks
volatile uint16_t tasks_100ms_counter = 0;      // local counter for 100ms tasks
volatile uint16_t time_counter_logger = 0;      // for the timing of the logger


inline void Tasks_MainTaskLoop(void);
inline void Tasks_100us(void);
inline void Tasks_1ms(void);
inline void Tasks_10ms(void);
inline void Tasks_100ms(void);

void mainDataExtract(void);
double mainGetLoadBuck(uint8_t bits);
double mainGetLoadBoost(uint8_t bits);
void GetVoltages(void);

int main(void)
{
    init_fosc();
    init_timer1();
    init_gpio();

    Dev_Lcd_Init();
    Dev_Button_Init();
    App_Logger_Init();
    App_Proto24_Init();
    App_Proto24_GetData(&global_proto24data);
    //TODO: is that delay for sending the SYS_RESET information after about one second to make sure that the pic24 is also ready?
    //TODO: this could also be integrated in the App_Proto24_Task_10ms function where we have a more accurate timing
    __delay_ms(200); // due to heavy workload on ISR, delays take ~3.7 times longer than normal

    // the PROTO_SYS_RESET message was eliminated because the pic24 does reset itself too which it should not do
    //App_Proto24_Send(PROTO_SYS_RESET);

    GetVoltages();
    global_data.packet_counter = 0;

    //Dev_Lcd_WriteStringXY(0,0,"MICROCHIP  dsPIC");
    //Dev_Lcd_WriteStringXY(0,1,"  33CK256MP505  ");
    //Dev_Lcd_WriteString("\vMICROCHIP  dsPIC  33CK256MP505  ");    //would work too without XY
    
    _T1IP = 1;  // Set interrupt priority to one (cpu is running on ip zero)
    _T1IF = 0;  // Reset interrupt flag bit
    _T1IE = 1;  // Enable Timer1 interrupt
    
    //Dev_UART1_WriteStringBlocking("Booting up the DPSK3!!\r\n");;
            
    Tasks_MainTaskLoop();
    
    return (0);
}

//======================================================================================================================
//  @brief  Timer 1 interrupt routine for generating the main highres timing for all the tasks
//  @note   with this simple implementation we do not lose any tick from timer 1, even when the tasks in the main loop
//  @note   take longer than 100 탎
//======================================================================================================================
void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void)
{
    counter_100us_leader++;     //increment our counter, no tick gets lost this way
    _T1IF = 0;                  //clear Timer1 interrupt flag
}

inline void Tasks_MainTaskLoop(void)
{
    //TODO: should we implement a Watchdog that gets triggered in one of the Task-Routines?
    while (1)   //100탎 scheduler
    {
        if (counter_100us_leader != counter_100us_follower)
        {
            counter_100us_follower++;
            Tasks_100us();
        }
    }
}

//======================================================================================================================
//  @brief  the function Tasks_100us gets called every 100탎, put your things in it that need to be called that often
//  @note   make sure that calls don't get lost even if this function takes longer than 100탎 !!!!
//======================================================================================================================
inline void Tasks_100us(void)
{
    DBGPIN_TOGGLE; // Toggle DEBUG-PIN
    if (tasks_1ms_counter++ >= 10)
    {
        tasks_1ms_counter = 0; //reset toggle counter
        Tasks_1ms();
    }
}

//======================================================================================================================
//  @brief  the function Tasks_10ms gets called every 10 ms, put your things in it that need to be called regularly
//  @note   there can be some jitter here because it is not called by a timer interrupt
//======================================================================================================================
inline void Tasks_1ms(void)
{
    App_Proto24_Task_1ms();
    if(App_Proto24_IsNewDataAvailable())
    {
        App_Proto24_GetData(&global_proto24data);
        mainDataExtract();
    }

    if (tasks_10ms_counter++ >= 10)
    {
        tasks_10ms_counter = 0; //reset toggle counter
        Tasks_10ms();
    }
}

//======================================================================================================================
//  @brief  the function Tasks_10ms gets called every 10 ms, put your things in it that need to be called regularly
//  @note   there can be some jitter here because it is not called by a timer interrupt
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
//  @note   there can be some jitter here because it is not called by a timer interrupt
//======================================================================================================================
inline void Tasks_100ms(void)
{
    DBGLED_TOGGLE;              // Toggle debug LED
    App_Display_Task_100ms();   // calling the display application that contains the main state machine
    GetVoltages();
    
    time_counter_logger++;
    if (time_counter_logger == 8)
        App_Logger_LogData();
    else if (time_counter_logger == 16)
    {   
        App_Logger_LogProto24();
        time_counter_logger = 0;
    }
}


void mainDataExtract(void)
{
    global_data.temperature     = global_proto24data.temperature;
    global_data.load_buck       = mainGetLoadBuck(global_proto24data.load_status.buck_still);
    global_data.step_load_buck  = mainGetLoadBuck(global_proto24data.load_status.buck_blink);
    global_data.load_boost      = mainGetLoadBoost(global_proto24data.load_status.boost_still);
    global_data.step_load_boost = mainGetLoadBoost(global_proto24data.load_status.boost_blink);
    global_data.fault_ocp_buck  = global_proto24data.fault_status.fault_ocp_buck;
    global_data.fault_ovp_buck  = global_proto24data.fault_status.fault_ovp_buck;
    global_data.fault_ocp_boost = global_proto24data.fault_status.fault_ocp_boost;
    global_data.fault_ovp_boost = global_proto24data.fault_status.fault_ovp_boost;
    global_data.packet_counter++;
}

//returns 1/ohms, because the loads are in parallel
double mainGetLoadBuck(uint8_t bits)
{
    double retval = 0.0;
    if(bits & 4)
        retval = 1.0/6.65;
    if(bits & 2)
        retval = retval + 1.0/8.25;
    if(bits & 1)
        retval = retval + 1.0/33.0;
    return retval;
}

//returns 1/ohms
double mainGetLoadBoost(uint8_t bits)
{
    double retval = 0.0;
    if(bits & 4)
        retval = 1.0/150.0;
    if(bits & 2)
        retval = retval + 1.0/215.0;
    if(bits & 1)
        retval = retval + 1.0/499.0;
    return retval;
}

// TODO: This is a dummy functions to fake the ADC values as long as they are not implemented
double GetVoltageBuck(void)     //fake, just for testing
{
    return 1.5;
}
// TODO: This is a dummy functions to fake the ADC values as long as they are not implemented
double GetVoltageBoost(void)     //fake, just for testing
{
    return 2.3;
}
// TODO: This is a dummy functions to fake the ADC values as long as they are not implemented
double GetVoltageInput(void)     //fake, just for testing
{
    return 3.4;
}

void GetVoltages(void)
{
    global_data.voltage_buck  = GetVoltageBuck();
    global_data.voltage_boost = GetVoltageBoost();
    global_data.voltage_input = GetVoltageInput();
//    global_data.fault_reg_buck  = buck_event; 
//    global_data.fault_reg_boost = boost_event;
    global_data.fault_reg_buck  = false;             //TODO: fake, just for testing
    global_data.fault_reg_boost = false;             //TODO: fake, just for testing
}


