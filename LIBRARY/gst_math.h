/*
 * Filters.h
 *
 *  Created on: 18 апр. 2020 г.
 *      Author: Ксения
 */

#ifndef GST_MATH_H_
#define GST_MATH_H_

//#include "main.h"

float _round(volatile float number, int digits);

float array_averaging(int len, float array[len]);
float arrays_xy_averaging(int len, float x_array[len], float y_array[len]);
float disperssion(int len, float array[len]);

void linear_regression(int len, float X[len], float Y[len],
		               float* a, float* b);


float SMA_angle(int steps);
float last_ited_angle(int iterations);
float WMA_angle(int steps);
float _WMA_angle(int steps);

float NSF_angle_2points(float previous_value,float current_value,float delta);
float EMA_angle_2points(float prev,float next,float alpha);
void EMA_angle(int steps, float delta, float arr[steps]);


float _fabs(volatile float number);
int get_sign(float number);
float clamp(float value, float min, float max);

float SMA_gyro(int steps, int dt, int axis, float* prev_angle);
float SMA_acc(int steps, int axis);


#endif /* GST_MATH_H_ */
