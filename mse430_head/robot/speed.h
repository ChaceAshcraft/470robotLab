/*
 * speed.h
 *
 *  Created on: Mar 30, 2017
 *      Author: Kristian Sims
 */

#ifndef SPEED_H_
#define SPEED_H_

// Public variables
extern long k_p, k_i, k_d, int_cap, max_speed;

// Public function prototypes
void speed_controller_init();
void speed_controller_tick();
void speed_a_set_target(int);
void speed_b_set_target(int);

#endif /* SPEED_H_ */
