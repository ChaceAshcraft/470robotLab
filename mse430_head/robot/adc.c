/*
 * adc.c
 *
 *  Created on: Sept 07, 2017
 *      Author: Kristian Sims
 */

#include "adc.h"
#include <msp430.h>
#include "mse430.h"
#include "pins.h"

int adc_init() {

    // Internal reference, 8 clock sample, slow reference buffer, only
    // power reference when sampling, 2.5 V reference, reference on
    ADC10CTL0 = SREF_1 | ADC10SHT_1 | ADC10SR | REFBURST | REF2_5V | REFON;

    // A3 for input, ADC10SC trigger, /4 ADC clock divider, ACLK, no repeat
    ADC10CTL1 = INCH_3 | SHS_0 | ADC10DIV_3 | ADC10SSEL_1 | CONSEQ_0;

    // Analog input enable (not sure if this is needed since we have INCH)
    ADC10AE0 |= BATTERY_ADC;

    // Turn ADC10 on (not sure when exactly to do this)
    ADC10CTL0 |= ADC10ON;

    return 0;
}

int adc_sample() {

    // Start conversion
    ADC10CTL0 |= ENC | ADC10SC;

    // Wait until complete (fixme)
    while (!(ADC10CTL0 & ADC10IFG));

    return ADC10MEM;
}
