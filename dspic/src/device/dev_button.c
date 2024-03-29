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

#include <xc.h>
#include <stdbool.h>
#include "device/dev_button.h"

//======================================================================================================================
// usually that defines for the pins are made automatically from the microchip code generator
// we decided to do that locally to avoid conflicts with the MCC
#define BUTTON_SetHigh()          _LATD1 = 1
#define BUTTON_SetLow()           _LATD1 = 0
#define BUTTON_Toggle()           _LATD1 ^= 1
#define BUTTON_GetValue()         _RD1
#define BUTTON_SetDigitalInput()  _TRISD1 = 1
#define BUTTON_SetDigitalOutput() _TRISD1 = 0
//======================================================================================================================

#define BUTTON_PRESSED   0 // low level active button
#define BUTTON_RELEASED  1
#define SCAN_PERIOD      (10)   // 10ms
#define DEBOUNCE_TIME    (20 / SCAN_PERIOD)   // 20ms
#define LONG_PRESS_TIME  (1500 / SCAN_PERIOD) //1500ms !!max 2.5seconds !!

typedef struct
{
    uint8_t debouncer;  // debouncing timer
    uint8_t timer;      // long press timer
    uint8_t event;      // event to be passed to the calling function
    bool    state;      // previous state
} button_data_t;

static volatile button_data_t buttonData;

//======================================================================================================================
// @brief initializes the button device, needs to be called once at boot up before that device can be used
//======================================================================================================================
void Dev_Button_Init(void)
{
    BUTTON_SetDigitalInput();
    buttonData.debouncer = 0;
    buttonData.timer     = 0;
    buttonData.event     = 0;
    buttonData.state     = BUTTON_RELEASED;
}


//======================================================================================================================
// @brief this function returns the event of the button like specified in the Dev_BUTTON_EVENT_* events above
// @returns event of the button
//======================================================================================================================
uint8_t Dev_Button_GetEvent(void)
{
    uint8_t ret = buttonData.event;
    buttonData.event = DEV_BUTTON_EVENT_NONE;
    return ret;
}


//======================================================================================================================
// @brief this function evaluates the button status/events and needs to be called approximately every 10 ms
//======================================================================================================================
void Dev_Button_Task_10ms(void)
{
    bool input_state = BUTTON_GetValue();
    bool last_state = buttonData.state;

    if(input_state == last_state)
    {
        buttonData.debouncer = 0; // (re)start debouncing counter
        if((input_state == BUTTON_PRESSED) && (buttonData.timer < LONG_PRESS_TIME))
        {
            buttonData.timer++;
            if(buttonData.timer == LONG_PRESS_TIME)
                buttonData.event = DEV_BUTTON_EVENT_PRESSED_LONG;
        }
    }
    else
    {
        buttonData.debouncer++; // increment debouncing counter
        if(buttonData.debouncer == DEBOUNCE_TIME)
        {
            buttonData.state = input_state; //debounce time passed, then accept the new state
            buttonData.debouncer = 0;
            if((input_state == BUTTON_RELEASED) && (buttonData.timer < LONG_PRESS_TIME))
            {
                buttonData.event = DEV_BUTTON_EVENT_PRESSED_SHORT;
            }
            buttonData.timer = 0;
        }
    }
}


