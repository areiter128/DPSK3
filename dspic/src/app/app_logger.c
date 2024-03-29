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

#define __print_serial_size 64

/*
static inline void UartSendText(char *pstring)
{
    while (*pstring != '\0')
    {
        while(Dev_UART1_TransmitBufferIsFull());    //wait while transmit buffer is full
        Dev_UART1_Write(*pstring++);
    }
}
*/

#define PrintSerialInit()  do{  UartSendText("\n\r"); } while(0)
#define PrintSerial(...)   do{char __print_utils_string[__print_serial_size]; sprintf(__print_utils_string, __VA_ARGS__); Dev_UART1_WriteStringBlocking(__print_utils_string); } while(0)


void App_Logger_Init(void)
{
    Dev_UART1_Init();
}


void App_Logger_LogData(void)
{
    double volt2;
    PrintSerial("==================================\n\r");
    PrintSerial("Packet count = %d\n\r", global_data.packet_counter);
    PrintSerial("Vbuck  =  %2.2f V\n\r", global_data.voltage_buck);
    PrintSerial("Vboost = %2.2f V\n\r", global_data.voltage_boost);
    PrintSerial("Vinput =  %2.2f V\n\r", global_data.voltage_input);
    PrintSerial("Temp   = %d deg C\n\r",  global_data.temperature);
    volt2 = global_data.voltage_buck * global_data.voltage_buck;
    PrintSerial("P buck = %1.2f W, ", volt2 * global_data.load_buck);
    PrintSerial("step = %1.2f W\n\r", volt2 * global_data.step_load_buck);
    volt2 = global_data.voltage_boost * global_data.voltage_boost;
    PrintSerial("Pboost = %1.2f W, ", volt2 * global_data.load_boost);
    PrintSerial("step = %1.2f W\n\r", volt2 * global_data.step_load_boost);
    PrintSerial("Buck faults:  OCP %d, OVP %d, REG %d\n\r", global_data.fault_ocp_buck, global_data.fault_ovp_buck, global_data.fault_reg_buck);
    PrintSerial("Boost faults: OCP %d, OVP %d, REG %d\n\r", global_data.fault_ocp_boost, global_data.fault_ovp_boost, global_data.fault_reg_boost);
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
