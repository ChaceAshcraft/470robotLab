/*
 * speed.c
 *
 *  Created on: Mar 30, 2017
 *      Author: Kristian Sims
 */

#include "speed.h"
#include <msp430.h>
#include "mse430.h"
#include "motor.h"

#define QBITS 10
#define F2Q(x) ((long)(x << QBITS))  // Don't know why this won't compile...
#define I2Q(x) (((long) (x)) << QBITS)

volatile long speed_a_target = 0;
volatile long speed_b_target = 0;

long k_p, k_i, k_d, int_cap, max_speed;

// If we multiply a long by a fixed point, and want a long as a result, we
// have to remove the fractional bits after the multiplication
inline long Q_L_MUL_2_L(long a, long b) {
    long long c;
    c = (long long)(long)a * (long long)(long)b;    // Multiplication in C...
    return (long) (c >> QBITS);
}

void speed_controller_init() {
    k_p = I2Q(12);
    k_i = I2Q(1) >> 1;  // 0.5, but the F2Q thing wasn't working
    k_d = I2Q(0);
    int_cap = 1000000000;
    max_speed = 64;
}

void speed_controller_tick() {

    static long old_a_err = 0;
    static long old_b_err = 0;
    static long err_a_int = 0;
    static long err_b_int = 0;
    long err_a_der, err_b_der;
    long speed_a_err = speed_a_target - motor_a_rate;
    long speed_b_err = speed_b_target - motor_b_rate;

    err_a_der = speed_a_err - old_a_err;
    err_b_der = speed_b_err - old_b_err;

    old_a_err = speed_a_err;
    old_b_err = speed_b_err;

    err_a_int += speed_a_err;
    err_b_int += speed_b_err;

    if (int_cap > 0) {
        err_a_int = long_constrain(err_a_int, -int_cap, int_cap);
        err_b_int = long_constrain(err_b_int, -int_cap, int_cap);
    }

    motor_a_set_power((int)
            Q_L_MUL_2_L(k_p, speed_a_err) +
            Q_L_MUL_2_L(k_i, err_a_int) +
            Q_L_MUL_2_L(k_d, err_a_der) );
    motor_b_set_power((int)
            Q_L_MUL_2_L(k_p, speed_b_err) +
            Q_L_MUL_2_L(k_i, err_b_int) +
            Q_L_MUL_2_L(k_d, err_b_der) );
}

void speed_a_set_target(int val) {

    if (max_speed > 0)
        speed_a_target = long_constrain((long)val, -max_speed, max_speed);
    else
        speed_a_target = (long)val;
}

void speed_b_set_target(int val) {

    if (max_speed > 0)
        speed_b_target = long_constrain((long)val, -max_speed, max_speed);
    else
        speed_b_target = (long)val;
}
