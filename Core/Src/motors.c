/*
 * motors.c
 *
 *  Created on: Jul 19, 2021
 *      Author: lastl
 */

#include "main.h"
#include "motors.h"


// Functions
// Moves motor counterclockwise one step
void StepCounterTurn(void) {
	GPIOA->ODR &= ~0x08;	  //sets pin PA3 so motor moves counterclockwise
	GPIOC->ODR |= 0x01;	  //starts a pulse to the PC0 step pin so the motor will turn
	GPIOC->ODR &= ~0x01;	//ends pulse
	HAL_Delay(10);	//wait is in ms
}

// Moves motor counterclockwise 360 degrees
void StepCounterTurn360(void) {
	for(int i=0; i < 200; i++) {
		StepCounterTurn();
	}
}

// Moves motor counterclockwise for user-specified degrees
void StepCounterTurnDegree(int degrees) {
	int steps = degrees / 1.8; // number of steps = degrees / degrees per step
	for(int i=0; i < steps; i++) {
		StepCounterTurn();
	}
}

// Moves motor clockwise one step
void StepClockTurn(void) {
	GPIOA->ODR |= 0x08;	  //sets pin PA3 so motor moves clockwise
	GPIOC->ODR |= 0x01;	  //starts a pulse to the PC0 step pin so the motor will turn
	GPIOC->ODR &= ~0x01;	//ends pulse
	HAL_Delay(10);	//wait is in ms
}

// Moves motor clockwise 360 degrees
void StepClockTurn360(void) {
	for(int i=0; i < 200; i++) {
		StepClockTurn();
	}
}

// Moves motor clockwise for user-specified degrees
void StepClockTurnDegree(int degrees) {
	int steps = degrees / 1.8; // number of steps = degrees / degrees per step
	for(int i=0; i < steps; i++) {
		StepClockTurn();
	}
}

// Servo Motor function
/*
 * 0 degrees = duty of 25 (0.5ms)
 * 45 degrees = duty of 50 (1.0ms)
 * 90 degrees = duty of 75 (1.5ms)
 * 180 degrees = duty of 125 (2.5ms)
 */
void ServoControl(int duty) {
	htim1.Instance->CCR1 = duty;
}

// Rotates servo from 0 to 90 degrees incrementally
void Servo0to45(int motor_id) {
	for( int duty = 25; duty < 50; duty++) {
		if( motor_id == 0) {
			htim1.Instance->CCR1 = duty;
		}
		else if( motor_id == 1) {
			htim1.Instance->CCR1 = duty; //TODO: make second servo motor configuration
		}
		HAL_Delay(100);
	}
}

// Rotates servo from 90 to 0 degrees incrementally
void Servo45to0(int motor_id) {
	for( int duty = 50; duty < 25; duty--) {
		if( motor_id == 0) {
			htim1.Instance->CCR1 = duty;
		}
		else if( motor_id == 1) {
			htim1.Instance->CCR1 = duty; //TODO: make second servo motor configuration
		}
		HAL_Delay(100);
	}
}

// Turns to spice container specified
void TurnToSpice(int container_number, int degree_direction[]) {
	int degrees;	//finds amount of degrees needed to turn to spice container
	int direction = 0; //counterclockwise is 0, clockwise is 1

	// turn right if spice container 0, 1, 2, or 3
	if( container_number <= 3) {
		degrees  = 45*(container_number + 1);
		StepCounterTurnDegree(degrees);
	}
	else {
		degrees = 45 * (7 - container_number);
		StepClockTurnDegree(degrees);
		direction = 1;
	}

	degree_direction[0] = degrees;
	degree_direction[1] = direction;
}

// Turns back to "home base" from location of spice container
void TurnBack(int degree_direction[]) {
	int degrees = degree_direction[0];
	int direction = degree_direction[1];	//counterclockwise is 0, clockwise is 1

	if( direction == 1) {	//we need to go in direction 0 to counteract recent turn
		StepCounterTurnDegree(degrees);
	}
	else {
		StepClockTurnDegree(degrees);
	}
}

void Dispense(int cycles) {
	for( int i=0; i < cycles; i++) {
		// Opens and closes the spice door
		Servo0to45(0);
		HAL_Delay(500);
		Servo45to0(0);

		// Opens and closes the portioning container door
		Servo0to45(1);
		HAL_Delay(500);
		Servo45to0(1);

	}
}

/*
 * 1. Moves spice to container based on container number
 * 2. Dispenses for number of cycles
 * 3. Rotates back to "home base"
 */
void DispenseSpice(int container_number, int cycles) {
	int degree_direction[2];

	TurnToSpice(container_number, degree_direction); //rotates to user-specified spice

	Dispense(cycles);

	TurnBack(degree_direction);	//turns back to home base
}

