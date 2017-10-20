/*
 * motor.c
 *
 *  Created on: Apr 13, 2016
 *      Author: Kristian Sims
 */

#include "motor.h"
#include <msp430.h>
#include "mse430.h"
#include "pins.h"

#define ALL_MOTOR_IN (MOTOR_A_IN_1 | MOTOR_A_IN_2 | MOTOR_B_IN_1 | MOTOR_B_IN_2)
#define ENCODER_A (MOTOR_A_HALL_A | MOTOR_A_HALL_B)
#define ENCODER_B (MOTOR_B_HALL_A | MOTOR_B_HALL_B)
#define QUAD_DIR_A  (-1)
#define QUAD_DIR_B  1

const int max_power = 512;
int motor_a_power = 0;
int motor_b_power = 0;

volatile long motor_a_count = 0;
volatile long motor_b_count = 0;
volatile long motor_a_rate;
volatile long motor_b_rate;
volatile char quadrature_a;
volatile char quadrature_b;

int motor_init() {

    // Init pins
    P1SEL &= ~ENCODER_A;
    P1SEL2 &= ~ENCODER_A;
    P1DIR &= ~ENCODER_A;

    P2SEL &= ~(ALL_MOTOR_IN | ENCODER_B);
    P2SEL2 &= ~(ALL_MOTOR_IN | ENCODER_B);
    P2DIR |= ALL_MOTOR_IN;
    P2DIR &= ~ENCODER_B;

    // Set up TimerA0 -- SMCLK is configured to DCO by default)
    TA1CTL = TASSEL_2 | MC_1;       // Stopped to begin with
    TA1CCR0 = max_power;

    // Turn on brakes (config/design choice?)
    P2OUT |= ALL_MOTOR_IN;

    // Start quadrature machine
    quadrature_a = P1IN & ENCODER_A;
    quadrature_b = P2IN & ENCODER_B;
    P1IES |= P1IN & ENCODER_A;
    P2IES |= P2IN & ENCODER_B;
    P1IFG &= ~ENCODER_A;
    P2IFG &= ~ENCODER_B;
    P1IE |= ENCODER_A;
    P2IE |= ENCODER_B;

    return 0;
}

void motor_a_set_power(int power) {

    P2SEL &= ~(MOTOR_A_IN_1 | MOTOR_A_IN_2);
    motor_a_power = power;

    if (power == 0) {
        P2OUT |= MOTOR_A_IN_1 | MOTOR_A_IN_2;
    } else if (power >= max_power) {
        P2OUT |= MOTOR_A_IN_2;      // CCW
        P2OUT &= ~MOTOR_A_IN_1;
        motor_a_power = max_power;
    } else if (power <= -max_power) {
        P2OUT |= MOTOR_A_IN_1;      // CW
        P2OUT &= ~MOTOR_A_IN_2;
        motor_a_power = -max_power;
    } else if (power > 0) {
        P2OUT |= MOTOR_A_IN_2;
        P2SEL |= MOTOR_A_IN_1;
        TA1CCR1 = power;
        TA1CCTL1 = OUTMOD_3;
    } else if (power < 0) {
        P2OUT |= MOTOR_A_IN_1;
        P2SEL |= MOTOR_A_IN_2;
        TA1CCR1 = -power;
        TA1CCTL1 = OUTMOD_3;
    }
}

void motor_b_set_power(int power) {

    P2SEL &= ~(MOTOR_B_IN_1 | MOTOR_B_IN_2);
    motor_b_power = power;

    if (power == 0) {
        P2OUT |= MOTOR_B_IN_1 | MOTOR_B_IN_2;
    } else if (power >= max_power) {
        P2OUT |= MOTOR_B_IN_1;      // CW (reverse for robot to go straight)
        P2OUT &= ~MOTOR_B_IN_2;
        motor_b_power = max_power;
    } else if (power <= -max_power) {
        P2OUT |= MOTOR_B_IN_2;      // CCW
        P2OUT &= ~MOTOR_B_IN_1;
        motor_b_power = -max_power;
    } else if (power > 0) {
        P2OUT |= MOTOR_B_IN_1;
        P2SEL |= MOTOR_B_IN_2;
        TA1CCR2 = power;
        TA1CCTL2 = OUTMOD_3;
    } else if (power < 0) {
        P2OUT |= MOTOR_B_IN_2;
        P2SEL |= MOTOR_B_IN_1;
        TA1CCR2 = -power;
        TA1CCTL2 = OUTMOD_3;
    }
}

void motor_update_rates() {
    static signed long old_a_count = 0;
    static signed long old_b_count = 0;

    motor_a_rate = motor_a_count - old_a_count;
    motor_b_rate = motor_b_count - old_b_count;

    old_a_count = motor_a_count;
    old_b_count = motor_b_count;
}

// Motor A quadrature encoder
#pragma vector=PORT1_VECTOR
__interrupt void port1_isr() {
    if (P1IFG & MOTOR_A_HALL_A) {
        switch (quadrature_a) {
        case 0:
            motor_a_count += QUAD_DIR_A;
            break;
        case MOTOR_A_HALL_A:
            motor_a_count -= QUAD_DIR_A;
            break;
        case MOTOR_A_HALL_B:
            motor_a_count -= QUAD_DIR_A;
            break;
        case ENCODER_A:
            motor_a_count += QUAD_DIR_A;
            break;
        }
        P1IES ^= MOTOR_A_HALL_A;
        P1IFG &= ~MOTOR_A_HALL_A;
    } else if (P1IFG & MOTOR_A_HALL_B) {
        switch (quadrature_a) {
        case 0:
            motor_a_count -= QUAD_DIR_A;
            break;
        case MOTOR_A_HALL_A:
            motor_a_count += QUAD_DIR_A;
            break;
        case MOTOR_A_HALL_B:
            motor_a_count += QUAD_DIR_A;
            break;
        case ENCODER_A:
            motor_a_count -= QUAD_DIR_A;
            break;
        }
        P1IES ^= MOTOR_A_HALL_B;
        P1IFG &= ~MOTOR_A_HALL_B;
    }
    quadrature_a = P1IN & ENCODER_A;    // Always read to not lose track
}

// Motor B quadrature encoder
#pragma vector=PORT2_VECTOR
__interrupt void port2_isr() {
    if (P2IFG & MOTOR_B_HALL_A) {
        switch (quadrature_b) {
        case 0:
            motor_b_count += QUAD_DIR_B;
            break;
        case MOTOR_B_HALL_A:
            motor_b_count -= QUAD_DIR_B;
            break;
        case MOTOR_B_HALL_B:
            motor_b_count -= QUAD_DIR_B;
            break;
        case ENCODER_B:
            motor_b_count += QUAD_DIR_B;
            break;
        }
        P2IES ^= MOTOR_B_HALL_A;
        P2IFG &= ~MOTOR_B_HALL_A;
    } else if (P2IFG & MOTOR_B_HALL_B) {
        switch (quadrature_b) {
        case 0:
            motor_b_count -= QUAD_DIR_B;
            break;
        case MOTOR_B_HALL_A:
            motor_b_count += QUAD_DIR_B;
            break;
        case MOTOR_B_HALL_B:
            motor_b_count += QUAD_DIR_B;
            break;
        case ENCODER_B:
            motor_b_count -= QUAD_DIR_B;
            break;
        }
        P2IES ^= MOTOR_B_HALL_B;
        P2IFG &= ~MOTOR_B_HALL_B;
    }
    quadrature_b = P2IN & ENCODER_B;
}

