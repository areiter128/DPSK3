/*
 * File:   init_adc.c
 * Author: M91406
 *
 * Created on July 9, 2019, 1:35 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "init_adc.h"


volatile uint16_t init_adc_module(void) {
    
    // ADCON1L: ADC CONTROL REGISTER 1 LOW
    ADCON1Lbits.ADON = 0; // ADC Enable: ADC module is off during configuration
    
    // Run basic initialization of ADC module features
    ADCON1Lbits.ADSIDL = 0; // ADC Stop in Idle Mode: Continues module operation in Idle mode
    ADCON1Hbits.SHRRES = 0b11; // Shared ADC Core Resolution Selection: 12-bit resolution ADC resolution = 12-bit (0...4095 ticks)
    ADCON1Hbits.FORM = 0; // Fractional Data Output Format: Integer

    ADCON2Lbits.SHRADCS = ADCON2_SHRADCS_DIV_MIN; // shared core clock divider = 2:1 (minimum)
    ADCON2Lbits.SHRSAMC = ADCON2_SHRSAMC_0008; // 10x TAD sampling time 
    ADCON2Lbits.REFCIE = ADCON2_REFCIE_OFF; // no error interrupt is triggered when Vref becomes available
    ADCON2Lbits.REFERCIE = ADCON2_REFERCIE_OFF; // no error interrupt is triggered when Vref fails
    ADCON2Lbits.REFERR = 0; // reset error flag
    ADCON2Lbits.REFRDY = 0; // reset bandgap status bit
    ADCON2Lbits.EIEN = ADCON2_EIEN_ON; // enable early interrupts
    ADCON2Lbits.SHREISEL = ADCON2_SHREISEL_8TAD; // ADC interrupts are triggered 8 TAD clocks ahead of completion

    ADCON3Lbits.CLKSEL = ADCON3_CLKSEL_FOSC; //ADCON3_CLKSEL_AFVCODIV; // set ADC clock to auxiliary clock (common clock with PWM)
    ADCON3Lbits.CLKDIV = ADCON3_CLKDIV_1; // set ADC input clock divider
    ADCON3Lbits.SHREN = ADCON3_CxEN_ENABLED; // enable shared ADC core
    ADCON3Lbits.CNVCHSEL = ADCON3_CNVCHSEL_AN0; // set AN0 to be target for software triggers (not used)
    ADCON3Lbits.CNVRTCH = ADCON3_CNVRTCH_READY; // reset TRIGGER READY flag bit
    ADCON3Lbits.REFSEL = ADCON3_REFSEL_AVDD_AVSS; // set ADC reference to AVDD-toAVSS
    ADCON3Lbits.SHRSAMP = ADCON3_SHRSAMP_HWTRIG; // use hardware trigger
    ADCON3Lbits.SUSPCIE = ADCON3_SUSPCIE_DISABLED; // disable ISR SUSPEND interrupt 
    ADCON3Lbits.SUSPEND = ADCON3_SUSPEND_RUN; // disable SUSPEND MODE
    ADCON3Lbits.SUSPRDY = ADCON3_SUSPRDY_RUNNING; // reset SUSPEND mode status flag bit
    ADCON3Lbits.SWCTRG = ADCON3_SWCTRG_READY; // set software trigger to READY
    ADCON3Lbits.SWLCTRG = ADCON3_SWLCTRG_LVLTRG_BY_HW; // set level sensitive triggers for hardware
    ADCON3Lbits.CNVCHSEL = ADCON3_CNVCHSEL_AN0; // configure the ADC inputs used (not used))
    
    ADCON4Hbits.C0CHS = ADCON4_C0CHS_S1AN0;
    ADCON4Hbits.C1CHS = ADCON4_C1CHS_S1AN1;
    ADCON4Lbits.SAMC0EN = ADCON4_SAMCxEN_DISABLED;
    ADCON4Lbits.SAMC1EN = ADCON4_SAMCxEN_DISABLED;

    ADCORE1Lbits.SAMC = 10;
    ADCORE1Hbits.RES = ADCON1_SHRRES_12BIT;
    ADCORE1Hbits.ADCS = ADCON2_SHRADCS_DIV_MIN; // shared core clock divider = 2:1 (minimum)
    ADCORE1Hbits.EISEL = ADCON2_SHREISEL_8TAD;
    
    ADCON5Hbits.SHRCIE = ADCON5L_CxCIE_DISABLED; // disable shared core to generate common interrupts
    ADCON5Lbits.SHRPWR = ADCON5_CxPWR_OFF; // Power off Shared Core => will be turned on later
    ADCON5Hbits.WARMTIME = ADCON5H_WARMTIME_CLK_32768; // Set warm-up time of ADC module
    
    
    return(1);
}

volatile uint16_t init_buck_adc(void) {
    
    
    
    return(1);
}

volatile uint16_t init_boost_adc(void) {
    
    
    
    return(1);
}

