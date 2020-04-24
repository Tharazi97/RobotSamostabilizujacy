/*
 * mojpid.c
 *
 *  Created on: 29.12.2018
 *      Author: Tharazi
 */
#include "mojpid.h"

float previous_error = 0;
float integral, derivative = 0;
float output=0;
float error, dt,kp,ki,kd;
void pid_init(float Kp, float Ki, float Kd, float T)
{
	kp=Kp;
	ki=Ki;
	kd=Kd;
	dt=T;
}

float pid_compute(float dane, float x)
{
	error = dane - x;
	integral = integral + error * dt;
	derivative = (error - previous_error) / dt;
	output = kp * error + ki * integral + kd * derivative;
	previous_error = error;
	if(output>100)output=100;
	if(output<-100)output=-100;
	return output;

}
