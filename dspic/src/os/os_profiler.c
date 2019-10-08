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
// @file os_profiler.c
//
// @brief contains functions to measure the maximum timing of some tasks
//
// @version v1.0
// @date 2019-08-29
// @author M52409
//
//=======================================================================================================

#include <string.h>
#include "os/os_profiler.h"
#include "os/os_scheduler.h"

void OS_Profiler_Init(os_profile_tasktiming* profile)
{
    memset(profile, 0, sizeof(os_profile_tasktiming));
}

void OS_Profiler_StartDurationMeasurement(os_profile_tasktiming* profile)
{
    OS_Scheduler_GetSystemTickValue(&(profile->time_start));
}

bool OS_Profiler_StopDurationMeasurement(os_profile_tasktiming* profile)
{
    OS_Scheduler_GetSystemTickValue(&(profile->time_stop));
    profile->time_task_duration = profile->time_stop - profile->time_start;
    if (profile->time_task_duration_max < profile->time_task_duration)
    {
        profile->time_task_duration_max = profile->time_task_duration;
        return true;
    }
    else
    {
        return false;
    }
}