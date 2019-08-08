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
// @file app_fault_handling.c
//
// @brief fault monitoring and handling
//
//======================================================================================================================

#include "stdbool.h"
#include "stdint.h"
#include "app/app_fault_handling.h"
#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_buck_custom.h"

typedef struct
{
    volatile uint16_t trip_cnt_threshold;           // Fault counter threshold triggering fault exception
    volatile uint16_t trip_cnt;                     // Fault trip counter
    volatile uint16_t reset_cnt_threshold;          // Fault counter threshold resetting fault exception
    volatile uint16_t reset_cnt;                    // Fault reset counter
}__attribute__((packed))FAULTBIT_CONDITION_SETTINGS_t;

typedef struct
{
    volatile uint16_t trip_level;                   // Input signal fault trip level/fault trip point
    volatile uint16_t trip_cnt_threshold;           // Fault counter threshold triggering fault exception
    volatile uint16_t trip_cnt;                     // Fault trip counter
    volatile uint16_t reset_level;                  // Input signal fault reset level/fault reset point
    volatile uint16_t reset_cnt_threshold;          // Fault counter threshold resetting fault exception
    volatile uint16_t reset_cnt;                    // Fault reset counter
}__attribute__((packed))FAULTVALUE_CONDITION_SETTINGS_t;


uint16_t active_faults = 0;

bool check_buckcurrent_peak = true;     //check buck current in peak or average mode
bool check_boostcurrent_peak = true;    //check boost current in peak or average mode

//  FAULT_CONDITION_SETTINGS_t:
//  uint16_t counter;                      // Fault hit counter
//  uint16_t trip_level;                   // Input signal fault trip level/fault trip point
//  uint16_t trip_cnt_threshold;           // Fault counter threshold triggering fault exception
//  uint16_t reset_level;                  // Input signal fault reset level/fault reset point
//  uint16_t reset_cnt_threshold;          // Fault counter threshold resetting fault exception

// voltages: in millivolt
// currents: in milliampere

// ==> buck overvoltage fault triggered after 5 milliseconds
// ==> buck overvoltage fault reset after 100 milliseconds
FAULTBIT_CONDITION_SETTINGS_t fault_handling_data_buck_overvoltage = {5, 0, 100, 0};

// ==> buck undervoltage fault triggered after 5 milliseconds
// ==> buck undervoltage fault reset after 100 milliseconds
FAULTBIT_CONDITION_SETTINGS_t fault_handling_data_buck_undervoltage = {5, 0, 100, 0};


// buck max. current is 1A ==> 1000 mA
// ==> buck peak current fault trip level   150% ==> 1500 mA for 10 milliseconds
// ==> buck peak current fault reset level  110% ==> 1100 mA for 100 milliseconds

// buck max. current is 1A ==> 1000 mA
// ==> buck avg. current fault trip level   105% ==> 1050 mA for 20 milliseconds
// ==> buck avg. current fault reset level  102% ==> 1020 mA for 100 milliseconds

//FAULT_CONDITION_SETTINGS_t  fault_handling_data_buck_voltage        = { 0, 3465, 20, 3366, 100 };
FAULTVALUE_CONDITION_SETTINGS_t  fault_handling_data_buck_peak_current   = { 0, 1500, 20, 1100, 100 };
FAULTVALUE_CONDITION_SETTINGS_t  fault_handling_data_buck_avg_current    = { 0, 1050, 20, 1020, 100 };

//uint16_t fh_buck_overvoltage_trip = 0;
uint16_t fh_buck_overvoltage_timer = 0;

// the boost voltage can be programmed from  10V to 17,8V,
// boost default programmed voltages is 15,0V ==> 15000 mV
// ==> boost voltage fault trip level        105% ==> 15750 mV for 20 milliseconds
// ==> boost voltage fault reset level       102% ==> 15300 mV for 100 milliseconds

// boost max. current is 0,2A ==> 200 mA
// ==> boost peak current fault trip level   150% ==> 300 mA for 10 milliseconds
// ==> boost peak current fault reset level  110% ==> 220 mA for 100 milliseconds

// boost max. current is 0,2A ==> 200 mA
// ==> boost avg. current fault trip level   105% ==> 210 mA for 20 milliseconds
// ==> boost avg. current fault reset level  102% ==> 204 mA for 100 milliseconds
FAULTVALUE_CONDITION_SETTINGS_t  fault_handling_data_boost_voltage       = { 0, 15750, 20, 15300, 100 };
FAULTVALUE_CONDITION_SETTINGS_t  fault_handling_data_boost_peak_current  = { 0,   300, 20,   220, 100 };
FAULTVALUE_CONDITION_SETTINGS_t  fault_handling_data_boost_avg_current   = { 0,   210, 20,   204, 100 };

//temperature: tenth degrees Celsius (TODO: or hundredth degrees???)
//
FAULTVALUE_CONDITION_SETTINGS_t  fault_handling_data_board_temperature   = { 0,   750, 100,  650, 100 };

// input voltage    minimum     typical     maximum
//                    6V          9V          13,8V     reset levels: +/- 5%
FAULTVALUE_CONDITION_SETTINGS_t  fault_handling_data_input_overvoltage   = { 0, 13800, 20, 13110, 100 };
FAULTVALUE_CONDITION_SETTINGS_t  fault_handling_data_input_undervoltage  = { 0,  6000, 20,  6300, 100 };

//======================================================================================================================
//  @brief  this function initializes the fault handling
//  @note   
//======================================================================================================================
void App_Fault_Handling_Init(void)
{
    //init the variables
    //todo
}

//======================================================================================================================
//  @brief  this function returns the fault status bits combined in one 16-bit word
//  @note   use the defined fault bits to filter out the status of a single fault
//  @note   bit zero (FAULT_GENERAL) of the result contains all the bits combined with an OR function
//======================================================================================================================
uint16_t App_Fault_Handling_GetFaults(void)
{
    return active_faults;
}

//======================================================================================================================
//  @TODO   Dummy functions need to be replaced !!!
//  @TODO   Dummy functions need to be replaced !!!
//  @TODO   Dummy functions need to be replaced !!!
//======================================================================================================================
uint16_t Dummy_GetSupplyVoltage(void)
{
    return 9000;     //9000 mV
}
uint16_t Dummy_GetBoardTemperatur(void)
{
    return 500;     //50,0 degree
}

uint16_t Dummy_GetBuckCurrent(void)
{
    return 500;     //500 mA
}
uint16_t Dummy_GetBoostVoltage(void)
{
    return 15000;     //15000 mV
}
uint16_t Dummy_GetBoostCurrent(void)
{
    return 200;     //200 mA
}


//======================================================================================================================
//  @brief  this function does the fault monitoring for one value
//  @note   it can handle maximum checks (like overvoltage) or minimum checks (like undervoltage)
//======================================================================================================================
void App_Fault_Handling_CheckFaultValue(FAULTVALUE_CONDITION_SETTINGS_t* faultdata, const uint16_t value, uint16_t faultbit)
{
    if (faultdata->trip_level > faultdata->reset_level)
    {
        //check for maximum
        if (value >= faultdata->trip_level)
        {
            faultdata->reset_cnt = 0;
            if (faultdata->trip_cnt < 65535)
                faultdata->trip_cnt++;
            if (faultdata->trip_cnt > faultdata->trip_cnt_threshold)
                active_faults |= faultbit;      //set fault bit
        }
        else if (value <= faultdata->reset_level)
        {
            faultdata->trip_cnt = 0;
            if (faultdata->reset_cnt < 65535)
                faultdata->reset_cnt++;
            if (faultdata->reset_cnt > faultdata->reset_cnt_threshold)
                active_faults &= ~faultbit;     //clear fault bit
        }
        else
        {
            //nothing to do with fault bits, leave them as they are
            faultdata->reset_cnt = 0;
            faultdata->trip_cnt = 0;
        }
    }
    else
    {
        //check for minimum
        if (value <= faultdata->trip_level)
        {
            faultdata->reset_cnt = 0;
            if (faultdata->trip_cnt < 65535)
                faultdata->trip_cnt++;
            if (faultdata->trip_cnt > faultdata->trip_cnt_threshold)
                active_faults |= faultbit;      //set fault bit
        }
        else if (value >= faultdata->reset_level)
        {
            faultdata->trip_cnt = 0;
            if (faultdata->reset_cnt < 65535)
                faultdata->reset_cnt++;
            if (faultdata->reset_cnt > faultdata->reset_cnt_threshold)
                active_faults &= ~faultbit;     //clear fault bit
        }
        else
        {
            //nothing to do with fault bits, leave them as they are
            faultdata->reset_cnt = 0;
            faultdata->trip_cnt = 0;
        }
    }
}

//======================================================================================================================
//  @brief  this function does the fault monitoring for one fault flag
//======================================================================================================================
void App_Fault_Handling_CheckFaultBit(FAULTBIT_CONDITION_SETTINGS_t* faultdata, bool faultflag, uint16_t faultbit)
{
    //check for maximum
    if (faultflag)
    {
        faultdata->reset_cnt = 0;
        if (faultdata->trip_cnt < UINT16_MAX)
            faultdata->trip_cnt++;
        if (faultdata->trip_cnt > faultdata->trip_cnt_threshold)
            active_faults |= faultbit;      //set fault bit
    }
    else
    {
        faultdata->trip_cnt = 0;
        if (faultdata->reset_cnt < UINT16_MAX)
            faultdata->reset_cnt++;
        if (faultdata->reset_cnt > faultdata->reset_cnt_threshold)
            active_faults &= ~faultbit;     //clear fault bit
    }
}


//======================================================================================================================
//  @brief  this function does the fault handling every 1 ms
//  @note   call this function in your main scheduler every 1ms
//======================================================================================================================
void App_Fault_Handling_Task_1ms(void)
{
    uint16_t    value;

    //check buck overvoltage and undervoltage
    App_Fault_Handling_CheckFaultBit(&fault_handling_data_buck_overvoltage, pwrCtrlBuck1_Data.flags.bits.overvoltage_fault, FAULT_BUCK_OVERVOLTAGE);
    App_Fault_Handling_CheckFaultBit(&fault_handling_data_buck_undervoltage, pwrCtrlBuck1_Data.flags.bits.undervoltage_fault, FAULT_BUCK_UNDERVOLTAGE);

    //check buck overcurrent
//    if (check_buckcurrent_peak)
//        App_Fault_Handling_CheckFaultValue(&fault_handling_data_buck_peak_current, value, FAULT_BUCK_OVERCURRENT);
//    else
//        App_Fault_Handling_CheckFaultValue(&fault_handling_data_buck_avg_current, value, FAULT_BUCK_OVERCURRENT);
    
    //check supply voltages:
    value = Dummy_GetSupplyVoltage();
    App_Fault_Handling_CheckFaultValue(&fault_handling_data_input_overvoltage, value, FAULT_SUPPLY_OVERVOLTAGE);
    App_Fault_Handling_CheckFaultValue(&fault_handling_data_input_undervoltage, value, FAULT_SUPPLY_UNDERVOLTAGE);    
    
    
    //check board temperature:
    value = Dummy_GetBoardTemperatur();
    App_Fault_Handling_CheckFaultValue(&fault_handling_data_board_temperature, value, FAULT_OVERTEMPERATURE);
    

    //check boost voltage and current
    value = Dummy_GetBoostVoltage();
    App_Fault_Handling_CheckFaultValue(&fault_handling_data_boost_voltage, value, FAULT_BOOST_OVERVOLTAGE);
    value = Dummy_GetBoostCurrent();
    if (check_boostcurrent_peak)
        App_Fault_Handling_CheckFaultValue(&fault_handling_data_boost_peak_current, value, FAULT_BOOST_OVERCURRENT);
    else
        App_Fault_Handling_CheckFaultValue(&fault_handling_data_boost_avg_current, value, FAULT_BOOST_OVERCURRENT);

    //set or clear the general fault bit
    if (active_faults & ~FAULT_GENERAL) //is one of the fault bits active?
        active_faults |= FAULT_GENERAL;
    else
        active_faults &= ~FAULT_GENERAL;
}


