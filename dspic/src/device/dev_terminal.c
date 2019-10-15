//=======================================================================================================
// Copyright(c) 2019 Microchip Technology Inc. and its subsidiaries.
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
// @file dev_terminal.c
//
// @brief functions to communicate with a serial terminal (like teraterm)
//
// @author M52409
//
// @date 2019-10-10
//
//=======================================================================================================

#include <stdio.h>
#include <stdarg.h>
#include "device/dev_uart1.h"
#include "device/dev_terminal.h"


//#define PrintSerial(...)   do{char __print_utils_string[TEMPSTR_SERIAL_SIZE]; sprintf(__print_utils_string, __VA_ARGS__); Dev_UART1_WriteStringBlocking(__print_utils_string); } while(0)

void Dev_Terminal_CursorHome(void)
{
    Dev_UART1_WriteStringBlocking("\033[1;1H");
}

void Dev_Terminal_ClearScreen(void)
{
    Dev_UART1_WriteStringBlocking("\033[2J");
}

#define TERMINAL_TEMPSTR_SIZE    160

extern int	vsnprintf(char *str, size_t size, const char *format, va_list ap);

void Dev_Terminal_Printf(char *fmt, ...)
{
    va_list args;
    char tmpString[TERMINAL_TEMPSTR_SIZE];
    
    va_start(args, fmt);
    //sprintf(tmpString, __VA_ARGS__);
    vsnprintf(tmpString, TERMINAL_TEMPSTR_SIZE, fmt, args);
    va_end(args);
    Dev_UART1_WriteStringBlocking(tmpString);
}

void Dev_Terminal_CursorGoto(uint8_t x, uint8_t y)
{
    Dev_Terminal_Printf("\033[%d;%dH", y+1, x+1);
}
