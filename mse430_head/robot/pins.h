/*
 * pins.h
 *
 *  Created on: Feb 27, 2016
 *      Author: Kristian Sims
 */

#ifndef PINS_H_
#define PINS_H_

// Port 1
#define RED_LEDS        BIT0    // Drive low to light LEDs on top
#define UART_RX         BIT1    // RX to BT TX
#define UART_TX         BIT2    // TX to BT RX
#define BUTTON_1        BIT3    // Button (not reset button)
#define INT             BIT3    // Interrupt from accelerometer, shared
#define BATTERY_ADC     BIT3    // Also shared with button and interrupt
#define MOTOR_A_HALL_B  BIT4    // Motor A encoder output B
#define MOTOR_A_HALL_A  BIT5    // Motor A encoder output A
#define I2C_SCL         BIT6    // UCB0SCL to Accelerometer SCL
#define I2C_SDA         BIT7    // UCB0SDA to Accelerometer SDA

// Port 2
#define MOTOR_B_HALL_A  BIT0    // Motor B encoder output A
#define MOTOR_A_IN_2    BIT1    // Motor A H-bridge input 2
#define MOTOR_A_IN_1    BIT2    // Motor A H-bridge input 1
#define MOTOR_B_HALL_B  BIT3    // Motor B encoder output B
#define MOTOR_B_IN_2    BIT4    // Motor B H-bridge input 2
#define MOTOR_B_IN_1    BIT5    // Motor B H-bridge input 1
#define XIN             BIT6    // 32.768 kHz crystal
#define XOUT            BIT7    // 32.768 kHz crystal

#endif /* PINS_H_ */
