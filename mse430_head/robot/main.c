/*
 * motor.c
 *
 *  Created on: Apr 13, 2016
 *      Author: Kristian Sims
 */

#include <msp430.h>
#include "mse430.h"
#include "uart.h"
#include "motor.h"
#include "speed.h"
#include "commands.h"

union message {
    char bytes[4];
    struct {
        int arg_a;
        int arg_b;
    } command;
} message;

int main(void) {

    // Initialize everything
    mse430_init();

    // Loop forever
    while (1) {
        // Avoid nasty race conditions
        __disable_interrupt();

        // Run speed controller
        if (tick_flag) {
            __enable_interrupt();
            tick_flag = 0;
            if (controller_on)
                speed_controller_tick();
        } else {
            __enable_interrupt();
            command_event();    // Process incoming commands
            __bis_SR_register(LPM0_bits | GIE);
        }
    }
}
