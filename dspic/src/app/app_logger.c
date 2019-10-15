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
// @file app_logger.c
//
// @brief app for sending some data to the pc for testing and visualisation
//
//======================================================================================================================

#include <stdio.h>
#include "device/dev_uart1.h"
#include "misc/global.h"
#include "misc/helpers.h"
#include "device/dev_terminal.h"
#include "driver/power_controllers/drv_power_controllers.h"

void App_Logger_Init(void)
{
    Dev_UART1_Init();
#ifndef TEST_ENABLED
    //PrintSerial("\033[2J");
    Dev_Terminal_ClearScreen();
#endif
}

void App_Logger_LogProto24(protocol_data_t *pData)
{
    uint8_t *pBin = (uint8_t *)pData;
    int i;
    PrintSerial("Raw PIC24 incoming data:");
    for(i = 0; i < sizeof(protocol_data_t); i++)
        PrintSerial(" %02x", pBin[i]);
    PrintSerial("\n\r");
}

extern POWER_CONTROLLER_DATA_t pwrCtrlBoost1_Data;      // data instance for the boost converter
extern POWER_CONTROLLER_DATA_t pwrCtrlBuck1_Data;       // data instance for the buck converter


void App_Logger_Task_100ms(void)
{
    static uint8_t task_terminal_log_counter;
    double volt2;

    switch(task_terminal_log_counter)
    {
        case 0:
            Dev_Terminal_CursorHome();
            PrintSerial("==================================\n\r");
            PrintSerial("Packet count = %d\n\r", global_data.pic24_packet_counter);
            PrintSerial("Vbuck  =  %2.2f V\n\r", global_data.buck.output_voltage);
            PrintSerial("Vboost = %2.2f V\n\r", global_data.boost.output_voltage);
            PrintSerial("Vinput =  %2.2f V\n\r", global_data.board.input_voltage);
            PrintSerial("Temp   = %d deg C\n\r",  global_data.board.temperature);
            break;
            
        case 2:
            volt2 = global_data.buck.output_voltage * global_data.buck.output_voltage;            //TODO: seems to be wrong to me, P = U*I
            PrintSerial("P buck = %1.2f W, ", volt2 * global_data.buck.load);
            PrintSerial("step = %1.2f W\n\r", volt2 * global_data.buck.step_load);
            break;
            
        case 3:
            volt2 = global_data.boost.output_voltage * global_data.boost.output_voltage;          //TODO: seems to be wrong to me, P = U*I
            PrintSerial("Pboost = %1.2f W, ", volt2 * global_data.boost.load);
            PrintSerial("step = %1.2f W\n\r", volt2 * global_data.boost.step_load);
            break;
        
        case 4:
            PrintSerial("Buck faults:  OCP %d, OVP %d, REG %d\n\r",
            global_data.buck.fault_overcurrentprotection, global_data.buck.fault_overvoltageprotection, global_data.buck.fault_reg);
            PrintSerial("Boost faults: OCP %d, OVP %d, REG %d\n\r",
            global_data.boost.fault_overcurrentprotection, global_data.boost.fault_overvoltageprotection, global_data.boost.fault_reg);
            break;
            
        case 7:
            Dev_Terminal_CursorGoto(0, 13);
            App_Logger_LogProto24(&global_proto24data);
            break;

        case 10:
            Dev_Terminal_CursorGoto(0, 16);
            Dev_Terminal_Printf("Buck  UnderVoltageLimit raw: %5d \n\r", pwrCtrlBuck1_Data.UnderVoltageLimit);
            Dev_Terminal_Printf("Buck  OutputVoltage     raw: %5d \n\r", pwrCtrlBuck1_Data.voltageOutput);
            Dev_Terminal_Printf("Buck  OverVoltageLimit  raw: %5d \n\r", pwrCtrlBuck1_Data.OverVoltageLimit);
            break;
        case 14:
            Dev_Terminal_CursorGoto(0, 20);
            Dev_Terminal_Printf("Boost UnderVoltageLimit raw: %5d \n\r", pwrCtrlBoost1_Data.UnderVoltageLimit);
            Dev_Terminal_Printf("Boost OutputVoltage     raw: %5d \n\r", pwrCtrlBoost1_Data.voltageOutput);
            Dev_Terminal_Printf("Boost OverVoltageLimit  raw: %5d \n\r", pwrCtrlBoost1_Data.OverVoltageLimit);
            break;
    }
    if (++task_terminal_log_counter > 20)
        task_terminal_log_counter = 0;
}
