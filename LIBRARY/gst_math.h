/*
 * Filters.h
 *
 *  Created on: 18 апр. 2020 г.
 *      Author: Ксения
 */

#ifndef GST_MATH_H_
#define GST_MATH_H_

//#include "main.h"
/**
 * @brief  Rounds of float number based on type cast
 * @param  number: number to round
 * @param  digits: number of significant digits
 * @retval rounded float number
 */
float _round(volatile float number, int digits);
/**
 * @brief  Returns averaged value of array:
 * aver=sum(array elements)/array length
 * @param  len: array length
 * @param  array[len]: array to average
 * @retval averaging result
 */
float array_averaging(int len, float array[len]);
/**
 * @brief  Returns value of averaged composition of two arrays:
 * xy=sum(array1_elements*array2_elements)/array length
 * @param  len: array length
 * @param  x_array[len]: first array
 * @param  y_array[len]: second array
 * @retval averaged composition result
 */
float arrays_xy_averaging(int len, float x_array[len], float y_array[len]);
/**
 * @brief  Returns dispersion of array:
 * disp=sum(array_elements*array_elements)/array length-@ref array_averaging(array)*array_averaging(array)
 * @param  len: array length
 * @param  len: array length
 * @param  array[len]: array from get dispersion
 * @retval dispersion of array
 */
float disperssion(int len, float array[len]);
/**
 * @brief  Calculates linear regression coefficients A and B from two arrays
 * based on @ref array_averaging(), @ref arrays_xy_averaging(), @ref disperssion()
 * Y=A*X+B
 * disp=sum(array_elements*array_elements)/array length-@ref array_averaging(array)*array_averaging(array)
 * @param  len: both arrays length
 * @param  X[len]: first array  (parameter)
 * @param  Y[len]: second array (function)
 * @param *a: pointer to variable, where A value stored
 * @param *b: pointer to variable, where B value stored
 */

void linear_regression(int len, float X[len], float Y[len],
		               float* a, float* b);
/**
 * @brief  Calculates simple moving average of measured degree value
 * from potentiometer degree
 * SMA+= @ref get_actual_angle()
 * @param  steps: measurement steps
 * @retval: SMA value of degree
 */
float SMA_angle(int steps);
/**
 * @brief  Returns last measured degree value
 * from potentiometer degree
 * for(iterations)
 * {last value = @ref get_actual_angle()}
 * @param  iterations: amount of measurements
 * @retval: last value of potentiometer degree
 */
float last_ited_angle(int iterations);
/**
 * @brief  Calculates weighted moving average of measured degree value
 * from potentiometer degree, most significant value is first
 * WMA+= [@ref get_actual_angle() * (steps-i))]x2/[(steps)*(steps+1)]
 * @param  steps: measurement steps
 * @retval: WMA value of degree
 */
float WMA_angle(int steps);
/**
 * @brief  Calculates weighted moving average of measured degree value
 * from potentiometer degree, most significant value is last
 * _WMA+= [@ref get_actual_angle() * (i+1))]x2/[(steps)*(steps+1)]
 * @param  steps: measurement steps
 * @retval: _WMA value of degree
 */
float _WMA_angle(int steps);

/**
 * @brief  Calculates weighted part of two values
 * from potentiometer degree:
 * result=(alpha)*previous_value+(1-alpha)*value
 * @param  prev: previous value
 * @param  next: current value
 * @param  alpha: alpha from formula above
 * @retval: value from formula above
 */
float EMA_angle_2points(float prev,float next,float alpha);
/**
 * @brief  Calculates exponential moving average of measured degree value
 * from potentiometer degree, based on @ref EMA_angle_2points()
 * @param  steps: measurement steps
 * @param  delta: result=(delta)*previous_value+(1-delta)*value
 * @param  arr[steps]: measurement steps
 * @retval: EMA value of degree
 */
void EMA_angle(int steps, float delta, float arr[steps]);


//float _fabs(volatile float number);
int get_sign(float number);



#endif /* GST_MATH_H_ */
