/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "gst_math.h"
#include "sd_hal_mpu6050.h"
#include "gst_pid.h"
#include "dwt_delay.h"
#include "USB_comunication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define X_AXIS 1    /*! Axis x for spatial data */
#define Y_AXIS 2    /*! Axis y for spatial data */
#define Z_AXIS 3    /*! Axis z for spatial data */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*! MPU6050 data structure */
SD_MPU6050 mpu1;
/*! MPU6050 communication result */
SD_MPU6050_Result result;
/*! MPU6050 temperature */
float mpu_temperature;
/*! Raw gyroscope data */
int16_t g_x, g_y, g_z;
/*! Raw accelerometer data */
int16_t a_x, a_y, a_z;
/*! potentiometer data */
uint16_t ADC1_DATA[1] = {0};
/*! linear regression coefficients*/
/*! deg = b*res+ a, RTD - resistance to degree*/
//y=b*x+a
float RTD_b=71.831;
float RTD_a=-109.200;

//float RTD_b=34.874;
//float RTD_a=34.131;
/*! linear regression coefficients*/
/*! CCRx_value = b*deg+ a, CCRxTD - CCRx to degree*/
/*! CCRx -  capture and compare register of PWM(TIM4)*/
float CCRxTD_a=1500;
float CCRxTD_b=ONE_DEG;

//float RTD_b=34.874;
//float RTD_a=34.131;


//float _DTR_b = 0.029;
//float _DTR_a =-0.097;
//
//float _RTD_b = 34.874;
//float _RTD_a = 34.131;

/*! angles from accelerometer*/
float angle_gyro_x, angle_gyro_y, angle_gyro_z = 0;
/*! gyro angles structure variable*/
ANG_GYROtypedef gyro_angles;
/*! complementary filter coefficient */
float CFK = 0.75;
/*! imu angle variable for saving imu angle data*/
/*! here is data from complementary filter*/
float a = 0;
/*! PID structure variable*/
PIDtypedef gst_pid;
/*! P coefficient of PID*/
float Kp = 0.86;
/*! I coefficient of PID*/
float Ki = 0.0;
/*! D coefficient of PID*/
float Kd = 0.0;
//float Ki = 0.10;
//float Kd = 0.95;

//float Kp = 1.68;
//float Ki = 0.04;
//float Kd = 0.03;
/*! Hold position in degrees for PID*/
float pid_hold = 0.0;

uint8_t uart_buffer[100]={0};   /*! buffer for uart data */
uint8_t uart_rx_counter = 0;    /*! counter for uart data */

/*! idn is string which is sent to uart at every program start*/
char* idn = "Camera gyrostabilization app.\r\nv3.0\r\nDate:13.05.2020\r\nAuthor:Kagirina K.A.";
/*! uart_string is served for saving commands from uart*/
char uart_string[100] ="";

/*! debug  data*/
uint32_t startTick;
uint32_t new_tick;
uint32_t tick, last_tick, dt;
//float temp[50], accx[50], accy[50]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/**
 * @brief  Gets microseconds from DWT timer
 * @param  startTick: time at program start point
 * @retval time since startTick in microseconds
 */
int get_microseconds_since_begin(uint32_t startTick);
/**
 * @brief  Gets milliseconds from DWT timer
 * @param  startTick: time at program start point
 * @retval time since startTick in milliseconds
 */
int get_milliseconds_since_begin(uint32_t startTick);
/*SERVO*/
/**
 * @brief  Writes number in CCRx register
 * @param  CCRx_value: number to write
 */
void servo_set_CCRx_value(uint16_t CCRx_value);
/**
 * @brief  Turns servo on some degrees.
 * For this purpose calculates CCRx value
 * from number degree to turn. Calculations based
 * on fuction CCRx=a*deg+b
 * @param  degree: count degrees to turn
 * @retval if degree in range [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE]
 * returns calculated CCRx value, else returns -1
 */
int servo_set_position(float degree);
/*test bench*/
/**
 * @brief  Calculates coefficients for degree in dependent from resistance.
 * Gets array of degrees, rotates servo on degrees from array
 * measures resistances at this degrees and saves their in array of resistances.
 * Then calculates coefficients from linear regression approximation algorithm.
 */
void potentiometer_calibration();
/**
 * @brief  Calculates coefficients for CRRx values in dependent from degrees.
 * Gets array of degrees, calculates CCRx array values from formula:
 * CCRs[i] = CCRx_ZERO_DEG+deg*ONE_DEG. (Naturally in this case, linear regression
 * coefficients equals CCRx_ZERO_DEG and ONE_DEG respectively)
 * Then calculates coefficients from linear regression approximation algorithm.
 */
void servo_calibration();
/**
 * @brief  Algorithm of servo position hold based on PID regulator
 * @param  *PID_struct: PID data structure
 * @param  *time_step: step for gyroscope angle calculations
 * @param  *timer: current time
 * @retval returns out = P(err)+I(err)+D(err) in degrees
 */
float stabilize_by_pid(PIDtypedef *PID_struct,
		               uint32_t time_step, uint32_t timer);
/**
 * @brief  Sets servo angle based on potentiometer data
 * with defined accuracy
 * @param  angle: angle value to set
 * @param  accuracy: desired accuracy of setting
 * @retval returns set angle in degrees
 */
float set_angle(float angle, float accuracy);
/**
 * @brief  Calculates angle of bench based on imu data and
 * complementary filter: a = (1-k)*ag+k*ac, ag - angle from
 * gyro and ac - angle from acc in one axis
 * @param  axis: axis
 * @param  CFK: k from formula above
 * @param *g_struct: gyro angle structure
 * @retval calculated angle from formula above in degrees
 */
float complementary_filter(int axis, float* CFK,
		              ANG_GYROtypedef *g_struct);
/*Potentiometer*/
/**
 * @brief  Calculates resistance from ADC data
 * @retval resistance value in volts
 */
float get_resistance();

//void measure();
//float stabilize_v1(float hold_angle, float prev_angle, float precision,
//		                             uint32_t time_step, uint32_t timer);
//float average_ADC(int steps);
/*UART*/


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  DWT_Init();
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC1_DATA, 1);
//  potentiometer_calibration();
//  servo_calibration();
//  HAL_UART_Transmit(&huart1, hello_string, strlen(hello_string), 10);
  UART_Transmit_string(&huart1, idn, 10);
  servo_set_position(SERVO_MIN_ANGLE);

  result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,
		   SD_MPU6050_Accelerometer_2G, SD_MPU6050_Gyroscope_250s);
  result = SD_MPU6050_SetDataRate(&hi2c1,&mpu1,
		                          SD_MPU6050_DataRate_8KHz);
  HAL_Delay(500);


  gyro_angles=*GYRO_struct_init(&gyro_angles);

  servo_set_position(pid_hold);
  a =pid_hold;
  last_tick=0;
  //startTick = DWT->CYCCNT;
  gst_pid = *PID_init(&gst_pid, &Kp, &Ki, &Kd, &pid_hold);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//  HAL_Delay(1);



	  if(strcmp(uart_string, "gyro:z\r") == 0)
	  {

		  a = angle_from_gyro(Z_AXIS, &gyro_angles);
		  tick=get_milliseconds_since_begin(gyro_angles.start_tick);
		  if(tick%500 != 0)
		  {
			  continue;
		  }
		  UART_Transmit_string(&huart1, "gz:\t", 10);
		  UART_send_float(&huart1, a, 10);
	  }

	  if(strcmp(uart_string, "stop_posting:\r") == 0)
	  {
		  strcpy(uart_string, "");
	  }

	  if(strcmp(uart_string, "calibr:start\r") == 0)
	  {
		  potentiometer_calibration();
		  strcpy(uart_string, "");
	  }

	  if(strcmp(uart_string, "calibr:send_ab\r") == 0)
	  {
		  UART_Transmit_string_CR_LF(&huart1, "resistance to degree a and b:", 10);
		  UART_send_float(&huart1,RTD_a,10);
		  UART_send_float(&huart1,RTD_b,10);
		  strcpy(uart_string, "");

	  }

	  if(strcmp(uart_string, "start:calibration\r") == 0)
	  {
		  potentiometer_calibration();
		  strcpy(uart_string, "");
	  }

	  if(strcmp(uart_string, "tracking:start\r") == 0)
	  {
		  tick=get_microseconds_since_begin(startTick);
		  a = stabilize_by_pid(&gst_pid, ONE_US*1000, tick);
		  UART_send_float(&huart1, a, 10);
	  }

	  if(strcmp(uart_string, "tracking:stop\r") == 0)
	  {
		  strcpy(uart_string, "");
	  }

	  if(uart_buffer[uart_rx_counter-1] == '\r')
	  {

		  strcpy(uart_string, uart_buffer);
		  uart_rx_counter=0;
		  clear_uart_buffer(uart_rx_counter, 100, uart_buffer);
		  HAL_Delay(1);
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void servo_set_CCRx_value(uint16_t CCRx_value)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,CCRx_value);
	HAL_Delay(25);
}

int servo_set_position(volatile float degree)
{
	if((degree>= SERVO_MIN_ANGLE) && (degree <= SERVO_MAX_ANGLE))
	{
		volatile uint16_t y = 0;
		y = (uint16_t)(CCRxTD_b*degree+CCRxTD_a);
		servo_set_CCRx_value(y);
//        HAL_Delay(50);
		return y;
	}
	else
	{
		return -1;
	}

}

void potentiometer_calibration()
{
	int len=ANGLE_RANGE*2;
	float ress[len];
	float degs[len];
	float degree = SERVO_MIN_ANGLE;

//	volatile float a,b, x_averaged, y_averaged, xy_averaged, Sx = 0;

    for(int d = 0; d<len; d++)
    {
    	degs[d]=degree;
  	    degree = degree + 0.5;
    }

	for(int d = 0; d < len; d++)
	{
		int err = servo_set_position(degs[d]);
		if(err != -1)
		{
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC1_DATA, 1);
			  ress[d]= (ADC1_DATA[0]*ADC1_K);

			  HAL_Delay(1);
		 }
         HAL_Delay(1);
	}


	linear_regression(len, ress, degs, &RTD_a, &RTD_b);
    HAL_Delay(10);
}

float get_resistance()
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC1_DATA, 1);
	volatile float res = ADC1_DATA[0]*ADC1_K;

	HAL_Delay(15);
	return res;
}

float get_actual_angle()
{

	volatile float current_resistance = get_resistance();
	volatile float current_angle = current_resistance*RTD_b + RTD_a;
	return current_angle;
}

float set_angle(float angle, float accuracy)
{


	volatile int direction = 1;
	volatile float step = 0.0;
	int i = 0;

	volatile float current_angle = get_actual_angle();
	volatile float delta = _fabs(angle - current_angle);
	volatile float position = current_angle + step;


	if(servo_set_position(angle) < 0)
	{
		return -2;
	}

	if(delta <= accuracy)
	{
		return current_angle;
	}

	while(1)
	{

		delta = _fabs(angle - current_angle);
		i++;
        direction = get_sign(angle - current_angle);

		if(delta >5)
		{
			step = 5*direction;
			current_angle = _WMA_angle(10);
		}

		if(delta<=5 && delta>accuracy)
		{

			step = 0.5*delta*direction;
			if((i%5)==0)
			{
				current_angle = _WMA_angle(100);
			}

			else
			{
				current_angle = current_angle+step;
			}
		}

		if(delta <= accuracy)
		{
			break;
		}
		position = current_angle + step;
		servo_set_position(position);

	}

	return current_angle;
}


float IMU_get_acc(int axis)
{
	  volatile float result = 0.0;
	  SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
 //     HAL_Delay(10);

	  if(axis == X_AXIS)
	  {
		  result=(float)mpu1.Accelerometer_X/MPU_AK;
	  }
	  if(axis == Y_AXIS)
	  {
		  result=(float)mpu1.Accelerometer_Y/MPU_AK;
	  }
	  if(axis == Z_AXIS)
	  {
		  result=(float)mpu1.Accelerometer_Z/MPU_AK;
	  }
	  return result;

}

float IMU_get_gyro(int axis)
{

	  SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
	  volatile float result = 0.0;
	  if(axis == X_AXIS)
	  {
		  result = (float)mpu1.Gyroscope_X/MPU_GK;
	  }
	  if(axis == Y_AXIS)
	  {
		  result =(float)mpu1.Gyroscope_Y/MPU_GK;
	  }
	  if(axis == Z_AXIS)
	  {
		  result = (float)mpu1.Gyroscope_Z/MPU_GK;
	  }
	  return result;

}

float IMU_get_temp()
{
	  SD_MPU6050_ReadTemperature(&hi2c1,&mpu1);
	  return mpu1.Temperature;
}


float angle_from_acc(int axis)
{
	volatile float accx,accy,accz;
	volatile float angle;


	accx = IMU_get_acc(X_AXIS);
	accy = IMU_get_acc(Y_AXIS);
	accz = IMU_get_acc(Z_AXIS);


		if (axis==X_AXIS)
		{
//			angle = sqrtf(accy*accy + accz*accz);
//			angle = accx/angle;
//			angle = RAD_TO_DEG*atanf(angle);
			angle = RAD_TO_DEG*atanf(accx/hypotf(accy, accz));
		}

		if (axis==Y_AXIS)
		{
//			angle = sqrtf(accx*accx + accz*accz);
//			angle = accy/angle;
//			angle = RAD_TO_DEG*atanf(angle);
			angle = RAD_TO_DEG*atanf(accy/hypotf(accx, accz));
		}

		if (axis==Z_AXIS)
		{
//			angle = sqrtf(accy*accy + accx*accx);
//			angle = angle/accz;
//			angle = RAD_TO_DEG*atanf(angle);
			angle = RAD_TO_DEG*atanf(accz/hypotf(accx, accy));
		}




	HAL_Delay(10);
	return angle;
}



float angle_from_gyro(int axis, ANG_GYROtypedef* g_struct)
{


	g_struct->time=get_microseconds_since_begin(g_struct->start_tick);
	volatile float dt = (float)((g_struct->time - g_struct->last_time)*1e-6);

	dt= fabs(dt);
	g_struct->last_time=get_microseconds_since_begin(g_struct->start_tick);

	if(axis == X_AXIS)
	{
		g_struct->ax=g_struct->ax+IMU_get_gyro(axis)*dt;
		return g_struct->ax;
	}

	else if(axis == Y_AXIS)
	{
		g_struct->ay=g_struct->ay+IMU_get_gyro(axis)*dt;
		return g_struct->ay;
	}

	else if(axis == Z_AXIS)
	{
		g_struct->az=g_struct->az+IMU_get_gyro(axis)*dt;
		return g_struct->az;
	}

}


float complementary_filter(int axis, float* CFK,
		              ANG_GYROtypedef* g_struct)
{


	volatile float k = *CFK;
	volatile float angle_acc, angle_gyro = 0;
    volatile float angle = 0;

	angle_acc = angle_from_acc(axis);
	angle_gyro = angle_from_gyro(axis, g_struct);

    angle= angle_gyro*(1-k)+angle_acc*k;
	return angle;
}

float stabilize_v1(float hold_angle, float prev_angle, float precision,
		                             uint32_t time_step, uint32_t timer)
{
	  volatile float da;
	  volatile float imu_angle;
	  if((timer%time_step)!=0)
	  {
		  return prev_angle;
	  }
	  imu_angle = complementary_filter( Z_AXIS, &CFK, &gyro_angles);

	  if(imu_angle < IMU_MAX_ANGLE)
	  {
		  imu_angle = IMU_MAX_ANGLE;
	  }

	  if(imu_angle > IMU_MIN_ANGLE)
	  {
		  imu_angle = IMU_MIN_ANGLE;
	  }

	  imu_angle = imu_angle*ITS_B + ITS_A;
	  da = fabs(prev_angle - imu_angle);
	  if(da < precision)
	  {
		  return imu_angle;
	  }

	  servo_set_position(imu_angle);
      return _WMA_angle(10);
//	  return set_angle(imu_angle, precision);
}

float stabilize_by_pid(PIDtypedef *PID_struct,
		               uint32_t time_step, uint32_t timer)
{
	  volatile float pid_error;
	  volatile float imu_angle;
	  volatile float set;

	  if((timer%time_step)!=0)
	  {
		  return PID_struct->out;
	  }
	  imu_angle = complementary_filter(Z_AXIS, &CFK, &gyro_angles);

	  if(imu_angle > IMU_MAX_ANGLE)
	  {
		  imu_angle = IMU_MAX_ANGLE;
	  }

	  if(imu_angle < IMU_MIN_ANGLE)
	  {
		  imu_angle = IMU_MIN_ANGLE;
	  }

	  imu_angle = imu_angle*ITS_B + ITS_A;
//	  pid_error = fabs(PID_struct->hold - imu_angle);
	  pid_error = PID_struct->hold - imu_angle;
	  PID(PID_struct, pid_error);

	  set = PID_struct->out;
//	  set = PID_struct->out;
	  servo_set_position(set);
	  return set;
	  //return _WMA_angle(50);
	  //return get_actual_angle();
}

void servo_calibration()
{
	  int MAX = 90;
	  int MIN = -90;
	  volatile float deg = (float)MIN;
	  volatile float step = 0.5;
	  uint16_t len = (MAX-MIN)/step;
	  float degs[len];
	  float CCRs[len];

	  for(uint16_t i = 0; i < len; i++)
	  {
		  degs[i]=deg;
//		  CCRs[i] = servo_set_position(deg);
		  CCRs[i] = CCRx_ZERO_DEG+deg*ONE_DEG;
		  deg = deg+step;
		  HAL_Delay(1);
	  }
	  linear_regression(len, degs, CCRs, &CCRxTD_a, &CCRxTD_b);
	  HAL_Delay(10);
}

void servo_calibration_evaluate()
{

    int i = 0;
    int len = (SERVO_MAX_ANGLE-SERVO_MIN_ANGLE)/0.5;
    float arr[len];
    servo_calibration();
	for(float a = SERVO_MIN_ANGLE; a <=SERVO_MAX_ANGLE; a = a+0.5)
	{
		arr[i] =servo_set_position(a);
		i++;
	}
}

int get_microseconds_since_begin(uint32_t startTick)
{
	  //new_tick = DWT->CYCCNT;
	  int tick= (int)DWT->CYCCNT - startTick;
	  int divider=(int)(SystemCoreClock/1e6);
	  tick=tick/divider;
	  return tick;
}

int get_milliseconds_since_begin(uint32_t startTick)
{
	  //new_tick = DWT->CYCCNT;
	  int tick= (int)DWT->CYCCNT - startTick;
	  int clock=(int)(SystemCoreClock/1e3);
	  tick=tick/clock;
	  return tick;
}

ANG_GYROtypedef* GYRO_struct_init(ANG_GYROtypedef* gyro_data)
{

	gyro_data->time = 0;
	gyro_data->last_time = 0;
	gyro_data->start_tick=DWT->CYCCNT;
//	gyro_data->us_dt=delay_us;
	gyro_data->ax = angle_from_acc(X_AXIS);
	gyro_data->ay = angle_from_acc(Y_AXIS);
	gyro_data->az = angle_from_acc(Z_AXIS);
	return gyro_data;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
