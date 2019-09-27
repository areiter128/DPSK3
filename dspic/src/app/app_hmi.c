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
// @file app_hmi.c
//
// @brief main statemachine for the human machine interface (buttons and display)
//
//=======================================================================================================

#include <stdint.h>
#include <stdio.h>
#include "device/dev_lcd.h"
#include "device/dev_button.h"
#include "app/app_fault_handling.h"
#include "misc/global.h"
#include "misc/helpers.h"

//=======================================================================================================
// defines
//=======================================================================================================


//=======================================================================================================
// @brief screen page defines
// @note these defines need to be customized to the needs of the application. every page define
// @note represents a screen page. put the defines for new screen pages here to use them in the functions
// @note CheckEvents and RefreshDisplay
//=======================================================================================================
#define PAGE_INIT_0                 0
#define PAGE_INIT_1                 1
#define PAGE_INIT_2                 2
#define PAGE_INIT_3                 3
#define PAGE_INIT_4                 4
#define PAGE_EXAMPLE_SHORT_BUTTON   5
#define PAGE_EXAMPLE_LONG_BUTTON    6
#define PAGE_VOLTAGES               7
#define PAGE_LOAD_BUCK              8
#define PAGE_LOAD_BOOST             9
#define PAGE_VIN_TEMP               10
#define PAGE_BUCK_FAULTS            11
#define PAGE_BOOST_FAULTS           12
#define PAGE_FAULT_HANDLING         13



#define HMI_AUTOREFRESHCOUNTER_100MS_MAX    8

//=======================================================================================================
// variables
//=======================================================================================================

uint16_t app_hmi_faultbits_copy = 0;
uint8_t app_hmi_pagestate = 0;
uint8_t app_hmi_lastpagestate = 0;
uint8_t app_hmi_autoresfreshcounter = HMI_AUTOREFRESHCOUNTER_100MS_MAX;
uint8_t app_hmi_pagetimeoutcounter = 0;


//=======================================================================================================
// prototypes
//=======================================================================================================
void CheckEvents(void);
void RefreshDisplay(void);
void GotoPage(uint8_t newpage);

//=======================================================================================================
// functions
//=======================================================================================================

//=======================================================================================================
//  @brief  this function contains the display task and needs to be called from outside every 100 ms
//  @note   the 100 ms interval does not need to be exact. it should be roughly 100 ms
//=======================================================================================================
void App_Hmi_Task_100ms(void)
{
    CheckEvents();
    
    if (++app_hmi_autoresfreshcounter >= HMI_AUTOREFRESHCOUNTER_100MS_MAX)
    {
        app_hmi_autoresfreshcounter = 0;
        RefreshDisplay();
    }    
}


//=======================================================================================================
//  @brief  this function checks for some events that can lead to jump to a different screen page
//  @note   put your code in here to scan button events or timing events or error events etc.
//=======================================================================================================
void CheckEvents(void)
{
    uint8_t buttonstate = Dev_Button_GetEvent();
    
    switch(app_hmi_pagestate)
    {
        case PAGE_INIT_0:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_VOLTAGES);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_FAULT_HANDLING);
            else if (App_Fault_Handling_GetFaults() != app_hmi_faultbits_copy)
                GotoPage(PAGE_FAULT_HANDLING);
            else if (++app_hmi_pagetimeoutcounter >= 20)
                GotoPage(PAGE_INIT_1);
            break;
        case PAGE_INIT_1:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_VOLTAGES);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_FAULT_HANDLING);
            else if (App_Fault_Handling_GetFaults() != app_hmi_faultbits_copy)
                GotoPage(PAGE_FAULT_HANDLING);
            else if (++app_hmi_pagetimeoutcounter >= 20)
                GotoPage(PAGE_INIT_2);           
            break;
        case PAGE_INIT_2:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_VOLTAGES);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_FAULT_HANDLING);
            else if (App_Fault_Handling_GetFaults() != app_hmi_faultbits_copy)
                GotoPage(PAGE_FAULT_HANDLING);
           else if (++app_hmi_pagetimeoutcounter >= 20)
                GotoPage(PAGE_INIT_3);           
            break;
        case PAGE_INIT_3:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_VOLTAGES);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_FAULT_HANDLING);
            else if (++app_hmi_pagetimeoutcounter >= 20)
                GotoPage(PAGE_INIT_4);           
            break;
        case PAGE_INIT_4:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_VOLTAGES);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_FAULT_HANDLING);
            else if (++app_hmi_pagetimeoutcounter >= 20)
                GotoPage(PAGE_INIT_0);           
            break;
/*            
        case PAGE_EXAMPLE_SHORT_BUTTON:
            if (buttonstate != DEV_BUTTON_EVENT_NONE)
                App_GotoPage(PAGE_INIT_0);
            break;
        case PAGE_EXAMPLE_LONG_BUTTON:
            if (buttonstate != DEV_BUTTON_EVENT_NONE)
                App_GotoPage(PAGE_INIT_0);
            break;
*/
        case PAGE_VOLTAGES:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_LOAD_BUCK);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_FAULT_HANDLING);
            break;
        case PAGE_LOAD_BUCK:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_LOAD_BOOST);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_VOLTAGES);
            break;
        case PAGE_LOAD_BOOST:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_VIN_TEMP);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_LOAD_BUCK);
            break;
        case PAGE_VIN_TEMP:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_BUCK_FAULTS);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_LOAD_BOOST);
            break;
        case PAGE_BUCK_FAULTS:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_BOOST_FAULTS);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_VIN_TEMP);
            break;
        case PAGE_BOOST_FAULTS:
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_FAULT_HANDLING);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_BUCK_FAULTS);
            break;
        case PAGE_FAULT_HANDLING:
            app_hmi_faultbits_copy = App_Fault_Handling_GetFaults();
            if (buttonstate == DEV_BUTTON_EVENT_PRESSED_SHORT)
                GotoPage(PAGE_INIT_0);
            else if (buttonstate == DEV_BUTTON_EVENT_PRESSED_LONG)
                GotoPage(PAGE_BOOST_FAULTS);
            break;
    }
}


//=======================================================================================================
//  @brief  this function renders the display content regularly 
//  @note   after changing to a different page it will be called immediately or else after a certain
//  @note   interval. put your display code for every page here
//=======================================================================================================
void RefreshDisplay(void)
{
    switch(app_hmi_pagestate)
    {
        case PAGE_INIT_0:
            Dev_Lcd_WriteStringXY(0,0,"   MICROCHIP    ");
            Dev_Lcd_WriteStringXY(0,1," TECHNOLOGY INC ");
            break;
        case PAGE_INIT_1:
            Dev_Lcd_WriteStringXY(0,0," DIGITAL POWER  ");
            Dev_Lcd_WriteStringXY(0,1," STARTER KIT 3  ");
            break;
        case PAGE_INIT_2:
            Dev_Lcd_WriteStringXY(0,0,"     dsPIC      ");
            Dev_Lcd_WriteStringXY(0,1,"  33CK256MP505  ");
            break;
        case PAGE_INIT_3:
            Dev_Lcd_WriteStringXY(0,0," Firmware: v1.0 ");
            Dev_Lcd_WriteStringXY(0,1," Hardware: v3.0 ");
            break;
        case PAGE_INIT_4:
            Dev_Lcd_WriteStringXY(0,0,"Press [USER] to ");
            Dev_Lcd_WriteStringXY(0,1,"   continue...  ");
            break;
/*            
        case PAGE_EXAMPLE_SHORT_BUTTON:
            Dev_Lcd_WriteStringXY(0,0," EXAMPLE PAGE 1 ");
            Dev_Lcd_WriteStringXY(0,1,"S2 PRESSED SHORT");
            break;
        case PAGE_EXAMPLE_LONG_BUTTON:
            Dev_Lcd_WriteStringXY(0,0," EXAMPLE PAGE 2 ");
            Dev_Lcd_WriteStringXY(0,1,"S2 PRESSED LONG");
            break;
*/
        case PAGE_VOLTAGES:
            PrintLcd(0, "Vbuck  =  %2.2f V ", global_data.buck.output_voltage);
            PrintLcd(1, "Vboost = %2.2f V ", global_data.boost.output_voltage);
            break;
        case PAGE_LOAD_BUCK:
            {
                double volt2 = global_data.buck.output_voltage * global_data.buck.output_voltage;     //TODO: seems to be wrong to me, P = U*I
                PrintLcd(0, "P buck = %1.2f W ", volt2 * global_data.buck.load);
                PrintLcd(1, "step   = %1.2f W ", volt2 * global_data.buck.step_load);
            }
            break;
        case PAGE_LOAD_BOOST:
            {
                double volt2 = global_data.boost.output_voltage * global_data.boost.output_voltage;   //TODO: seems to be wrong to me, P = U*I
                PrintLcd(0, "Pboost = %1.2f W    ", volt2 * global_data.boost.load);
                PrintLcd(1, "step   = %1.2f W    ", volt2 * global_data.boost.step_load);
            }
            break;
        case PAGE_VIN_TEMP:
            if(global_data.board.input_voltage < 10.0) 
            { PrintLcd(0,"Vin  =   %2.2f V  ", global_data.board.input_voltage); }
            else
            { PrintLcd(0,"Vin  =  %2.2f V   ", global_data.board.input_voltage); }
            PrintLcd(1,"Temp = %d deg C   ", global_data.board.temperature);
            break;
        case PAGE_BUCK_FAULTS:
            PrintLcd(0,"Buck faults:    ");
            PrintLcd(1,"OC %d OV %d REG %d ", global_data.buck.fault_overcurrentprotection,
                    global_data.buck.fault_overvoltageprotection, global_data.buck.fault_reg);
            break;
        case PAGE_BOOST_FAULTS:
            PrintLcd(0,"Boost faults:   ");
            PrintLcd(1,"OC %d OV %d REG %d ", global_data.boost.fault_overcurrentprotection,
                    global_data.boost.fault_overvoltageprotection, global_data.boost.fault_reg);
            break;
        case PAGE_FAULT_HANDLING:
            PrintLcd(0,"Fault bits:     ");
            Dev_Lcd_GotoXY(0,1);
            {
                uint8_t i;
                uint16_t fault_bits;
                fault_bits = App_Fault_Handling_GetFaults();
                for (i=0; i<8; i++)
                {
                    if (fault_bits & (1<<i))
                        Dev_Lcd_WriteChar('1');
                    else
                        Dev_Lcd_WriteChar('0');
                    Dev_Lcd_WriteChar(' ');
                }
            }
            break;
        default:
            PrintLcd(0," FIRMWARE ERROR ");
            PrintLcd(1,"  PRESS [RESET] ");
            break;
    }
}


//=======================================================================================================
//  @brief  this function is useful for jumping to a different page to display
//  @note   the new page will be displayed immediately 
//=======================================================================================================
void GotoPage(uint8_t newpage)
{
    //sometimes some states require to know where we came from (and maybe want to jump back)
    app_hmi_lastpagestate = app_hmi_pagestate;
    app_hmi_pagestate = newpage;
    app_hmi_autoresfreshcounter = HMI_AUTOREFRESHCOUNTER_100MS_MAX;
    app_hmi_pagetimeoutcounter = 0;
}
