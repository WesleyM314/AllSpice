/*
 * motors.h
 *
 *  Created on: Jul 19, 2021
 *      Author: lastl
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

// Function prototypes
// Stepper motors
void StepCounterTurn();
void StepCounterTurn360();
void StepCounterTurnDegree(int degrees);
void StepClockTurn();
void StepClockTurn360();
void StepClockTurnDegree(int degrees);

// Servo motors
void ServoControl(int duty);
void Servo0to45(int motor_id);
void Servo45to0(int motor_id);
void TurnToSpice(int container_number, int degree_direction[]);
void TurnBack(int degree_direction[]);
void Dispense(int cycles);
// vvv Wesley calls this vvv
void DispenseSpice(int container_number, int cycles);


#endif /* INC_MOTORS_H_ */
