/*
 * gst_math.c
 *
 *  Created on: 18 апр. 2020 г.
 *      Author: Ксения
 */

#include "main.h"
#include "gst_math.h"

float _round(volatile float number, int digits)
{
	volatile int tens = 1;
	for (int i = 0; i<digits; i++)
	{
		tens =10*tens;
	}
	number = (int)(number*tens);
	number = (float)(number/tens);
	return number;
}

float array_averaging(int len, float array[len])
{
	volatile float averaged = 0;
	for(int i = 0; i < len; i++)
	{
		averaged += array[i];
	}
	averaged = averaged/(float)len;
	return averaged;
}

float arrays_xy_averaging(int len, float x_array[len], float y_array[len])
{
	volatile float xy = 0;
	for(int i = 0; i < len; i++)
	{
		xy += (x_array[i]*y_array[i]);

	}
	xy = xy/(float)len;
	return xy;
}

float disperssion(int len, float array[len])
{
	volatile float S, averaged = 0;
	for(int i = 0; i < len; i++)
	{
		S += (array[i]*array[i]);
		HAL_Delay(1);
	}

	averaged = array_averaging(len,array);

	S = (S/len - (averaged*averaged));
	HAL_Delay(1);

	return S;
}

void linear_regression(int len, float X[len], float Y[len],
		               float* a, float* b)
{
	volatile float res_a, res_b, x_averaged, y_averaged, xy_averaged, Sx = 0;

	x_averaged = array_averaging(len,X);

    y_averaged = array_averaging(len,Y);

    xy_averaged = arrays_xy_averaging(len,X,Y);

    Sx = disperssion(len,X);

    res_b = (xy_averaged - x_averaged*y_averaged)/Sx;

    res_a = y_averaged - res_b*x_averaged;

    *a = res_a;
    *b = res_b;
    HAL_Delay(1);

}


//simple moving average
float SMA_angle(int steps)
{
	float temp = 0.0;
	for(int i=0; i<steps; i++)
	{
		temp = temp + get_actual_angle();
	}
	temp = temp/steps;
	return temp;
}


float last_ited_angle(int iterations)
{
	float temp = 0.0;
	for(int i=0; i<iterations; i++)
	{
		temp = get_actual_angle();
	}
	return temp;
}
//weighted moving average
float WMA_angle(int steps)
{
	float temp = 0.0;
	for(int i=0; i<steps; i++)
	{
		temp = temp + (get_actual_angle())*(steps-i);
	}

	temp = temp*2;
	temp = temp/steps;
	temp = temp/(steps+1);
	return temp;
}


float _WMA_angle(int steps)
{
	float temp = 0.0;
	for(int i=0; i<steps; i++)
	{
		temp = temp + (get_actual_angle())*(i+1);
	}

	temp = temp*2;
	temp = temp/steps;
	temp = temp/(steps+1);
	return temp;
}


//float noise spike filter
float NSF_angle_2points(float previous_value,float current_value,float delta)
{

	float angle;
	if((current_value-current_value)== delta)
	{
		angle = current_value;
	}

	if((current_value-previous_value)> delta)
	{
		angle = previous_value + delta;
	}

	if((current_value-previous_value)< delta)
//	if((previous_value-current_value)> delta)
	{
		angle = previous_value - delta;
	}

	return angle;
}



float EMA_angle_2points(float prev,float next,float alpha)
{
	float angle=-1;
	if( (alpha<=1) && (alpha >=0))
    {
    	angle = alpha*prev+(1-alpha)*next;
    }

	if( (alpha>1) || (alpha < 0))
    {
		alpha = 0.5;
		angle = alpha*prev+(1-alpha)*next;
    }

	return angle;
}

void EMA_angle(int steps, float delta, float arr[steps])
{
	for(int i=0; i<steps; i++)
	{
		arr[i] = get_actual_angle();
	}

	for(int i=0; i<steps; i++)
	{
		if(i != 0)
		{
			arr[i] = EMA_angle_2points(arr[i-1],arr[i],delta);
		}
	}
}

float _fabs(volatile float number)
{
	number = (number>= 0) ? number :-number;
	return number;
}

int get_sign(float number)
{

	if(number > 0)
	{
		return 1;
	}
	else if(number < 0)
	{
		 return -1;
	}

	else
	{
		return 0;
	}

}

float clamp(float value, float min, float max)
{
//	volatile float result;
//	if(value > max)
//	{
//		result = max;
//	}
//
//	if(value<min)
//	{
//		result= min;
//	}
//
//	if((value<=max) && (value>=min))
//	{
//		result = value;
//	}
//		return result;
	volatile float result;
	if(value > max)
	{
		result = max;
	}

	else if(value < min)
	{
		result = min;
	}
	else
	{
		result = value;
	}

	return result;
}



float SMA_acc(int steps, int axis)
{
	volatile float temp = 0.0;
	for(int i=0; i<steps; i++)
	{
		temp+=IMU_get_acc(axis);
//		HAL_Delay(10);
	}
	temp = temp/steps;
	return temp;
}

