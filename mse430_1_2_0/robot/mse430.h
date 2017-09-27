/*
 * mse430.h
 *
 *  Created on: Mar 29, 2017
 *      Author: Kristian Sims
 */

#ifndef MSE430_H_
#define MSE430_H_

// Public variables
extern volatile char tick_flag, controller_on;
extern volatile unsigned long ticks;

// Public function prototypes
void mse430_init();
int int_constrain(int, int, int);
long long_constrain(long, long, long);

#endif /* MSE430_H_ */
