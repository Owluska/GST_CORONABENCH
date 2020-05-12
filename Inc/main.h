/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	uint32_t start_tick;
	int last_time;
	int time;
//	uint32_t us_dt;
	float ax;
	float ay;
	float az;
}ANG_GYROtypedef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


#define ADC1_K (3.0/4096.0)
/*One bit of gyro ADC at +/- 250s*/
#define MPU_GK 131
/*One bit of acc ADC at +/- 2g*/
#define MPU_AK 16384
/* Radian to degree constant*/
#define RAD_TO_DEG 180/3.141593
/* time constant for complimentary filter */
#define ONE_US 1
/* Coefficient between CCRx value and servo degrees */
#define ONE_DEG 11.11111111
/*CCRx value of servo on zero degree*/
#define CCRx_ZERO_DEG 1500
/* Angle limits for servo */
#define SERVO_MAX_ANGLE 40
#define SERVO_MIN_ANGLE -40
/* Angle limits for IMU */
#define IMU_MAX_ANGLE SERVO_MAX_ANGLE
#define IMU_MIN_ANGLE SERVO_MIN_ANGLE
/**
 * @brief Coefficients for f(x) = x*b+a approximation
 * here x - imu angle, f(x) - servo angle
 */
#define ITS_B 1.0
#define ITS_A 0.0

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* Safe zone of servo movement */
#define ANGLE_RANGE (SERVO_MAX_ANGLE-SERVO_MIN_ANGLE)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**
 * @brief  Returns potentiometer value in degrees
 *         based on RTD_b and RTD_a values
 * @retval Angle value:
 */
float get_actual_angle();
/**
 * @brief  Gets accelerometer value on one axis
 * @param  axis: axis
 * @retval acc value in g
 */
float IMU_get_acc(int axis);
/**
 * @brief  Gets gyroscope value on one axis
 * @param  axis: axis
 * @retval acc value in rad/s
 */
float IMU_get_gyro(int axis);
/**
 * @brief  Gets IMU temperature value
 * @retval temperature value in degrees
 */
float IMU_get_temp();
/**
 * @brief  Calculates angle on one axis(x, y or z)
 * based on ACC data.
 * Angle is calculated from the projection
 * of acceleration data (in g) on the Z axis
 * @param  axis: axis
 * @retval angle value in degrees
 */
float angle_from_acc(int axis);
/**
 * @brief  Calculates angle on one axis(x, y or z)
 * based on gyro data.
 * Angle is calculated with this equation:
 * a=a(t=0)+integral[gyro_data*dt]
 * from Euler method: a = prev_a+a*dt
 * @param  axis: axis
 * @retval angle value in degrees
 */
float angle_from_gyro(int axis, ANG_GYROtypedef* g_struct);
ANG_GYROtypedef* GYRO_struct_init(ANG_GYROtypedef* gyro_data);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
