/*
 * mse430.c
 *
 *  Created on: Mar 29, 2017
 *      Author: Kristian Sims
 */

#include "mse430.h"
#include <msp430.h> 
#include "pins.h"
#include "uart.h"
#include "motor.h"
#include "speed.h"
#include "adc.h"

// Public variables
volatile char tick_flag = 0;
volatile char controller_on = 0;
volatile unsigned long ticks = 0;

// Local function prototypes
void mse430_clock_init();

void mse430_init() {

    // Initialize modules
    mse430_clock_init();
    uart_init();
    motor_init();
    speed_controller_init();
    // accel_init();
    adc_init();

    __enable_interrupt();

    // Other miscellaneous stuff
    P1DIR |= RED_LEDS;
    P1OUT &= ~RED_LEDS;
#ifdef BUTTON                   // TODO: Fix ISRs so this can work
    P1REN |= BUTTON_1;
    P1IE |= BUTTON_1;           // MOTOR_A_HALL_A | MOTOR_A_HALL_B;
    P1IES |= BUTTON_1;          // Falling edge
    P1IFG &= ~BUTTON_1;         // Clear spurious interrupts
#endif

    // Unused pins/ports
    P1SEL |= I2C_SCL | I2C_SDA;
    P1SEL2 |= I2C_SCL | I2C_SDA;
    P3DIR = 0xFF;
    P3OUT = 0x00;
}

void mse430_clock_init() {

    // Setup watchdog timer
    WDTCTL = WDT_ADLY_16;
    IE1 |= WDTIE;

    // Init clock - 16 MHz
    DCOCTL = DCO2 | DCO1 | DCO0;
    BCSCTL1 = XT2OFF | RSEL3 | RSEL2 | RSEL1 | RSEL0;
    // BCSCTL2 defaults to 0, which is what we want
    BCSCTL3 = XCAP_3;   // Internal 12.5 pF capacitors

    // Set up 32.768 kHz crystal
    P2SEL |= XIN | XOUT;
    P2DIR |= XIN | XOUT;

    // Crystal fault trap/wait
    while (BCSCTL3 & LFXT1OF)
        __bis_SR_register(LPM0_bits);   // WDT will wake it up
    IFG1 &= ~OFIFG;                     // Clear oscillator interrupt flag
}

int int_constrain(int in, int min, int max) {
    if (in < min)
        return min;
    else if (in > max)
        return max;
    else
        return in;
}

long long_constrain(long in, long min, long max) {
    if (in < min)
        return min;
    else if (in > max)
        return max;
    else
        return in;
}

#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR() {
    motor_update_rates();
    tick_flag = 1;
    ticks++;
    __bic_SR_register_on_exit(LPM0_bits);
}
