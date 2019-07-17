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
// @file app_fault_handling.h
//
// @brief fault monitoring and handling

//
//======================================================================================================================


#ifndef _APP_FAULT_HANDLING_H_
#define _APP_FAULT_HANDLING_H_

//#include <xc.h>
//#include <stdbool.h>
#include <stdint.h>
//#include <stdlib.h>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif

#define FAULT_GENERAL                   1
#define FAULT_SUPPLY_OVERVOLTAGE        2
#define FAULT_SUPPLY_UNDERVOLTAGE       4
#define FAULT_OVERTEMPERATURE           8
#define FAULT_BUCK_OVERVOLTAGE          16
#define FAULT_BUCK_OVERCURRENT          32
#define FAULT_BOOST_OVERVOLTAGE         64
#define FAULT_BOOST_OVERCURRENT         128

typedef struct
{
    volatile uint16_t trip_level;                   // Input signal fault trip level/fault trip point
    volatile uint16_t trip_cnt_threshold;           // Fault counter threshold triggering fault exception
    volatile uint16_t trip_cnt;                     // Fault trip counter
    volatile uint16_t reset_level;                  // Input signal fault reset level/fault reset point
    volatile uint16_t reset_cnt_threshold;          // Fault counter threshold resetting fault exception
    volatile uint16_t reset_cnt;                    // Fault reset counter
}__attribute__((packed))FAULT_CONDITION_SETTINGS_t;

//======================================================================================================================
//  @brief  this function initializes the fault handling
//  @note   
//======================================================================================================================
void App_Fault_Handling_Init(void);


uint16_t App_Fault_Handling_GetFaults(void);

//======================================================================================================================
//  @brief  this function does the fault handling every 1 ms
//  @note   call this function in your main scheduler every 1ms
//======================================================================================================================
void App_Fault_Handling_Task_1ms(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif

#endif  //_APP_FAULT_HANDLING_H_