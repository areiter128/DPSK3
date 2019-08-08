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
// @file drv_power_controllers.c
//
// @brief instances of the various power controllers that are needed in the project
//
// @ntoe    in this application we have two power converters
//          1. Buck converter
//          2. Boost converter
//          all the instances of the data structures for that converters are placed in this file
//
// @author M52409
//
// @date 2019-08-06
//=======================================================================================================

#include <stdbool.h>
#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_buck_custom.h"
#include "driver/power_controllers/drv_power_controller_buck_generic.h"

void Drv_PowerControllers_Init(void)
{
    // Init all Buck Converter instances
    Drv_PowerControllerBuck1_Init(true);                       // Init Buck Convert 1
    Drv_PowerControllerBuck1_SetOutputVoltageReference_mV(3300); //Set Buck Converter Output to 3.3 Volt

    // Init all Boost Converter instances
    //TODO: Boost needs to be implemented
    //Drv_PowerControllerBoost1_Init(true );                       // Init Boost Convert 1
    //Drv_PowerControllerBoost1_SetOutputVoltageReference_mV(15000); //Set Boost Converter Output to 15 Volt
}

void Drv_PowerControllers_Task_100us(void)
{
    Drv_PowerControllerBuck_Task_100us(&pwrCtrlBuck1_Data);
    //TODO:
    //Drv_PowerControllerBoost_Task_100us(&pwrCtrlBoost1_Data);
}
