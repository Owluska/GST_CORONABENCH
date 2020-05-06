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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//this values are related to PWM settings and properties
//#define SERVO_K 24.1667
//#define SERVO_B -566.6667



// these values are not related to calibration
// the user must get them from the boundaries
// within which the movement should be carried out
//#define SERVO_MAX_ANGLE 105
//#define SERVO_MIN_ANGLE 70

#define SERVO_MAX_ANGLE 40
#define SERVO_MIN_ANGLE -40

#define ADC1_K (3.0/4096.0)

#define MPU_GK 131
#define MPU_AK 16384

#define RAD_TO_DEG 180/3.141593

//acordingly to servo MIN and MAX angles
#define IMU_MAX_ANGLE SERVO_MAX_ANGLE
#define IMU_MIN_ANGLE SERVO_MIN_ANGLE
//f = kb+a
#define ITS_B 1.0
#define ITS_A 0.0

#define ONE_US 1
//how much CCRx in one degree
//max servo angle PWM period (4000),
//min servo angle PWM period (2000)
#define ONE_DEG 11.11111111
#define CCRx_ZERO_DEG 1500



//#define POT_K 0.019
//#define POT_B -1.274
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ANGLE_RANGE (SERVO_MAX_ANGLE-SERVO_MIN_ANGLE)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
float get_actual_angle();
float IMU_get_acc(int axis);
float IMU_get_gyro(int axis);
float IMU_get_temp();
float angle_from_acc(int axis);
float angle_from_gyro(int axis, float dt_us, float* previous_angle);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
