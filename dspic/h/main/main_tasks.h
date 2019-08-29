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
// @file main_scheduler.h
//
// @brief contains the main scheduler that calls all the tasks that need to be called regularly
//
// @note put your application specific tasks that need to be called regularly into main_scheduler.c
// @note call the Init and the RunForever-Function in your main.c
//
// @version v1.0
// @date 2019-08-02
// @author M52409
//
//=======================================================================================================


#ifndef _DEV_MAIN_SCHEDULER_H_
#define _DEV_MAIN_SCHEDULER_H_


#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif


extern void Main_Scheduler_Init(void);

//=======================================================================================================
//  @brief  Scheduler function for calling all the Tasks regularly ( 100us, 1ms, 10ms, 100ms, 1s )
//  @note   call this function as last function in main.c after calling the Init-function
//          please consider that the timing of the calls are dependent on the duration of the last call
//          the resulting jitter therefore depends on the timing of the calls before
//=======================================================================================================
extern void Main_Scheduler_RunForever(void);


#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif              // _DEV_MAIN_SCHEDULER_H_