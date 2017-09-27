/*
 * commands.c
 *
 *  Created on: May 11, 2017
 *      Author: Kristian Sims
 */

#include "commands.h"
#include <msp430.h>
#include "mse430.h"
#include "uart.h"
#include "motor.h"
#include "speed.h"
#include "adc.h"

void command_event() {

    // Data structure for received commands and set responses
    char tag;
    union {
        char bytes[4];
        int ints[2];
        long int32;
    } data;

    if (uart_bytes_pending() < 5)
        return;

    // Get data from Bluetooth
    tag = uart_get_char();
    uart_read(data.bytes, 4);

    switch (tag) {

    case 'P':                           // Set power
        motor_a_set_power(data.ints[0]);
        motor_b_set_power(data.ints[1]);
        controller_on = 0;
    case 'p':                           // Get power (fall-through)
        data.ints[0] = motor_a_power;
        data.ints[1] = motor_b_power;
        break;

    case 'S':                           // Set speed targets
        speed_a_set_target(data.ints[0]);
        speed_b_set_target(data.ints[1]);
        controller_on = 1;
    case 's':                           // Get speed values (fall-through)
        data.ints[0] = motor_a_rate;
        data.ints[1] = motor_b_rate;
        break;

    case 'K':                           // Set k_p (prop term, but P was taken)
        k_p = data.int32;
        break;
    case 'k':                           // Get k_p
        data.int32 = k_p;
        break;

    case 'I':                           // Set k_i (integral term)
        k_i = data.int32;
        break;
    case 'i':                           // Get k_i
        data.int32 = k_i;
        break;

    case 'D':                           // Set k_d (derivative term)
        k_d = data.int32;
        break;
    case 'd':                           // Get k_d
        data.int32 = k_d;
        break;

    case 'C':                           // Set int_cap (max value of integral)
        int_cap = data.int32;
        break;
    case 'c':                           // Get int_cap
        data.int32 = int_cap;
        break;

    case 'M':                           // Set max_speed
        max_speed = data.int32;
        break;
    case 'm':                           // Get max_speed
        data.int32 = max_speed;
        break;

    case 'b':                           // Check battery
        data.ints[0] = adc_sample();
        data.ints[1] = 0;
        break;

    default:                            // Bad Command
        tag = '?';
        data.int32 = 0;
        break;
    }

    uart_put_char(tag);
    uart_write(data.bytes, 4);
}
