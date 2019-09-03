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

#define ADC_POWRUP_TIMEOUT  5000

volatile uint16_t init_boost_adc(void) {
    
    // ANSELx: ANALOG SELECT FOR PORTx REGISTER
    ANSELDbits.ANSELD10 = 1; // Analog input is enabled and digital input is disabled for RD10 (Boost converter output voltage feedback)
    
    // ADLVLTRGL: ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGHbits.LVLEN18 = 0; // Input trigger is edge-sensitive
    
    // ADMOD0L: ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD1Lbits.DIFF18 = 0; // Differential-Mode for Corresponding Analog Inputs: Channel is single-ended
    ADMOD1Lbits.SIGN18 = 0; // Output Data Sign for Corresponding Analog Inputs: Channel output data are unsigned
    
    // ADEIEL: ADC EARLY INTERRUPT ENABLE REGISTER LOW
    ADEIEHbits.EIEN18 = 1; // Early interrupt is enabled for the channel
    
    // ADIEL: ADC INTERRUPT ENABLE REGISTER LOW
    ADIEHbits.IE18 = 1; // Common Interrupt Enable: Common and individual interrupts are disabled for the corresponding channel

    // ADTRIGnL/ADTRIGnH: ADC CHANNEL TRIGGER n(x) SELECTION REGISTERS LOW AND HIGH
    ADTRIG4Hbits.TRGSRC18 = 0b01010; // Trigger Source Selection for Corresponding Analog Inputs: PWM4 Trigger 1
    
    // ADCMPxCON: ADC DIGITAL COMPARATOR x CONTROL REGISTER
    ADCMP2CONbits.CHNL = 18; // Input Channel Number: 13=AN13
    ADCMP2CONbits.CMPEN = 0; // Comparator Enable: Comparator is disabled
    ADCMP2CONbits.IE = 0; // Comparator Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the comparator
    ADCMP2CONbits.BTWN = 0; // Between Low/High Comparator Event: Disabled
    ADCMP2CONbits.HIHI = 0; // High/High Comparator Event: Disabled
    ADCMP2CONbits.HILO = 0; // High/Low Comparator Event: Disabled
    ADCMP2CONbits.LOHI = 0; // Low/High Comparator Event: Disabled
    ADCMP2CONbits.LOLO = 0; // Low/Low Comparator Event: Disabled
    
    // ADCMPxENL: ADC DIGITAL COMPARATOR x CHANNEL ENABLE REGISTER LOW
    ADCMP2ENHbits.CMPEN18 = 0; // Comparator Enable for Corresponding Input Channels: AN18 Disabled
    
    // ADCMPxLO: ADC COMPARARE REGISTER LOWER THRESHOLD VALUE REGISTER
    ADCMP2LO = 1399; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 9Vin=1399 ADC ticks

    // ADCMPxHI: ADC COMPARARE REGISTER UPPER THRESHOLD VALUE REGISTER
    ADCMP2HI = 2799; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 18Vin=2799 ADC ticks
    
    // ADFLxCON: ADC DIGITAL FILTER x CONTROL REGISTER
    ADFL2CONbits.FLEN = 0; // Filter Enable: Filter is disabled
    ADFL2CONbits.MODE = 0b11; // Filter Mode: Averaging mode (always 12-bit result 7 in oversampling mode 12-16bit wide)
    ADFL2CONbits.OVRSAM = 0b001; // Filter Averaging/Oversampling Ratio: 16x (result in the ADFLxDAT)
    ADFL2CONbits.IE = 0; // Filter Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the filter
    ADFL2CONbits.FLCHSEL = 18; // Oversampling Filter Input Channel Selection: 18=AN18
    
    return(1);
}

