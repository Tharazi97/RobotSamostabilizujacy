/*
 * kalman.h
 *
 *  Created on: 29.12.2018
 *      Author: Tharazi
 */

#ifndef kalman_h
#define kalman_h

void KalmanFilter(double angle, double bias, double measure);
double KalmanUpdate(double newValue, double newRate);

#endif /* KALMAN_H_ */
