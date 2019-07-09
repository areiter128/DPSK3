/*
 * File:   main.c
 * Author: M91406
 *
 * Created on July 8, 2019, 1:52 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "device/dev_lcd.h"
#include "device/dev_button.h"
#include "app/app_display.h"
#include "main.h"


volatile uint16_t tasks_10ms_counter = 0; // local counter for 100ms tasks
volatile uint16_t tasks_100ms_counter = 0; // local counter for 100ms tasks
#define TMR1_TIMEOUT    30000   // Timeout protection for Timer1 interrupt flag bit

inline void Tasks_100us(void);
inline void Tasks_10ms(void);
inline void Tasks_100ms(void);

int main(void)
{
    volatile uint16_t timeout = 0;

    init_fosc();
    init_timer1();
    init_gpio();

    Dev_Lcd_Init();
    Dev_Button_Init();
    
    //Dev_Lcd_WriteString("\vMICROCHIP  dsPIC  33CK256MP505  ");    //would works too without XY
    //Dev_Lcd_WriteStringXY(0,0,"MICROCHIP  dsPIC");
    //Dev_Lcd_WriteStringXY(0,1,"  33CK256MP505  ");
    
    while (1)   //100탎 scheduler
    {
        //TODO: events can be lost with this implementation if the task takes longer than 100탎
        // wait for timer1 to overrun
        while ((!_T1IF) && (timeout++ < TMR1_TIMEOUT));
        timeout = 0;    // Reset timeout counter
        _T1IF = 0; // reset Timer1 interrupt flag bit
        Tasks_100us();
    }
    return (0);
}


//======================================================================================================================
//  @brief  the function Tasks_100us gets called every 100탎, put your things in it that need to be called that often
//  @note   make sure that calls don't get lost even if this function takes longer than 100탎 !!!!
//======================================================================================================================
inline void Tasks_100us(void)
{
    DBGPIN_TOGGLE; // Toggle DEBUG-PIN

    if (tasks_10ms_counter++ >= 100)
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
}

