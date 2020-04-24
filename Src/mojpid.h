/*
 * mojpid.h
 *
 *  Created on: 29.12.2018
 *      Author: Tharazi
 */

#ifndef MOJPID_H_
#define MOJPID_H_

/*
 * inicjalizacja regulatora PID
 * nastawy to Kp, Ki, Kd, stala probkowania
 */
void pid_init(float Kp, float Ki, float Kd, float T);

/*
 * przeliczenie danych przez regulator PID
 */
float pid_compute(float dane,float x);

#endif /* MOJPID_H_ */
