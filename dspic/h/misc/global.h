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
// @file global.h
//
// @brief global defines, structures and variables
//
//======================================================================================================================

#ifndef _GLOBAL_H_
#define	_GLOBAL_H_

#include <stdint.h>
#include "proto_data.h"

#ifdef	__cplusplus
extern "C" {
#endif


typedef struct
{
    double  voltage_buck;
    double  voltage_boost;
    double  voltage_input;
    double  load_buck;
    double  step_load_buck;
    double  load_boost;
    double  step_load_boost;
    uint8_t temperature;
    uint8_t packet_counter;
    uint8_t fault_ocp_buck:1;
    uint8_t fault_ovp_buck:1;
    uint8_t fault_ocp_boost:1;
    uint8_t fault_ovp_boost:1;
    uint8_t fault_reg_buck:1;
    uint8_t fault_reg_boost:1;
    uint8_t :2;
} print_data_t;

extern print_data_t global_data;
extern protocol_data_t global_proto24data;
//debug variables
//extern uint16_t global_debug_uart2_rx_counter;
//extern uint16_t global_debug_uart2_tx_counter;



#ifdef	__cplusplus
}
#endif

#endif  // _GLOBAL_H_