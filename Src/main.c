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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SD_MPU6050 mpu1;
SD_MPU6050_Result result;
float mpu_temperature;
int16_t g_x, g_y, g_z;
int16_t a_x, a_y, a_z;

uint8_t uart_buffer[100]={0};
uint8_t uart_rx_counter = 0;
uint8_t uart_rx_data = 0;

uint16_t ADC1_DATA[1] = {0};
//variables for calibration
//number 2 in array size means precision: step = 0.5 deg
//float just_angles[ANGLE_RANGE*2]={0.0};
//float sma_angles[ANGLE_RANGE*2]={0.0};
//float wma_angles[ANGLE_RANGE*2]={0.0};
//float _wma_angles[ANGLE_RANGE*2]={0.0};
//float nsf_angles[ANGLE_RANGE*2]={0.0};
//float ema_angles[ANGLE_RANGE*2]={0.0};
//float last_angles[ANGLE_RANGE*2]={0.0};

float gx[100]={0.0};
float gy[100]={0.0};
float gz[100]={0.0};

float ax[100]={0.0};
float ay[100]={0.0};
float az[100]={0.0};


float CFK = 0.95;

//float DTR_b,DTR_a=0.0;
//y=b*x+a
float RTD_b=34.874;
float RTD_a=34.131;

float CCRxTD_a=1500;
float CCRxTD_b=ONE_DEG;

//float RTD_b=34.874;
//float RTD_a=34.131;

float Kp = 1.68;
float Ki = 0.04;
float Kd = 0.03;
float pid_hold = 0;

//float _DTR_b = 0.029;
//float _DTR_a =-0.097;
//
//float _RTD_b = 34.874;
//float _RTD_a = 34.131;

char* hello_string = "Head program greets you\r\n";
char uart_string[100] ="";
//t in us
//int t = 0;
//dt in us
int dt = 1000;
float imu_angle=0;
float angle_gyro_x, angle_gyro_y, angle_gyro_z = 0;


PIDtypedef gst_pid;
uint32_t startTick;
uint32_t new_tick;
uint32_t tick;


float temp[50], accx[50], accy[50]={0};
//float accx[50]={0};
float a = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/*SERVO*/
int servo_set_position(float degree);
void servo_set_CCRx_value(uint16_t CCRx_value);
/*test bench*/
void potentiometer_calibration();
void servo_calibration();
void measure();
float stabilize_v1(float hold_angle, float prev_angle,
		   float precision, int time_step, int timer);
float stabilize_by_pid(PIDtypedef *PID_struct, int time_step, int timer);
//void get_r_a_function(int len, double angles[len], double resistances[len]);
//float get_actual_angle();
float set_angle(float angle, float accuracy);
float complementary_filter(float dt, int axis,
		         float* angle_gyro, float* CFK);
/*Potentiometer*/
float get_resistance();
float average_ADC(int steps);
/*UART*/
void send_series_(int len, float array[len]);
void UART_send_float(float number);
void clear_uart_buffer(uint8_t counter, uint8_t len, uint8_t buffer[len]);
/*IMU*/
//float IMU_get_acc(int axis);
//float IMU_get_gyro(int axis);
//float IMU_get_temp();
float angle_from_acc(int axis);

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
//  test_bench_calibration();
//  servo_calibration();
  HAL_UART_Transmit(&huart1, hello_string, strlen(hello_string), 10);
  servo_set_position(SERVO_MIN_ANGLE);

  result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,
		   SD_MPU6050_Accelerometer_2G, SD_MPU6050_Gyroscope_250s);
  result = SD_MPU6050_SetDataRate(&hi2c1,&mpu1,
		                          SD_MPU6050_DataRate_8KHz);
  HAL_Delay(500);

  angle_gyro_x = angle_from_acc(X_AXIS);
  angle_gyro_y = angle_from_acc(Y_AXIS);
  angle_gyro_z = angle_from_acc(Z_AXIS);

//  set_angle(pid_hold, 0.5);
  servo_set_position(pid_hold);
  a =pid_hold;
  startTick = DWT->CYCCNT;
  gst_pid = *PID_init(&gst_pid, &Kp, &Ki, &Kd, &pid_hold);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


     HAL_Delay(1);
     CFK=0.85;
	 for(int i = 0; i<(sizeof(temp)/sizeof(int)); i++ )
	 {
		temp[i]=complementary_filter(1000, Z_AXIS, &angle_gyro_z, &CFK);
		accy[i] = angle_from_gyro(Z_AXIS, 1000, &angle_gyro_z);
//		accx[i] = IMU_get_gyro(Z_AXIS);
	 }


	  if(strcmp(uart_string, "fil_test:start\r") == 0)
	  {
		  strcpy(uart_string, "");

		  double stp = 0.5;
		  for(double d = SERVO_MIN_ANGLE; d<=SERVO_MAX_ANGLE; d=d+stp)
		  {
			  servo_set_position(d);
			  UART_send_float(get_actual_angle());
		  }
	  }

	  if(strcmp(uart_string, "set_angle:debug\r") == 0)
	  {
		  strcpy(uart_string, "");

//		  int i = 0;
		  UART_Transmit_string_CR_LF(&huart1, "started testing setting angle", 10);
//		  for(double d = SERVO_MIN_ANGLE; d<=SERVO_MAX_ANGLE; d=d+0.5)
//		  {
//			  just_angles[i] = set_angle(d);
//			  UART_send_float(just_angles[i]);
//			  i++;
//		  }

//		  for(double d = SERVO_MAX_ANGLE; d>=SERVO_MIN_ANGLE; d=d-0.5)
//		  {
//			  just_angles[i] = set_angle(d);
//			  UART_send_float(just_angles[i]);
//			  i++;
//		  }
		  UART_Transmit_string_CR_LF(&huart1, "stopped testing setting angle", 10);
	  }

	  if(strcmp(uart_string, "calibr:start\r") == 0)
	  {
		  potentiometer_calibration();
		  strcpy(uart_string, "");
	  }

	  if(strcmp(uart_string, "calibr:send_ab\r") == 0)
	  {
		  UART_Transmit_string_CR_LF(&huart1, "resistance to degree a and b:", 10);
		  UART_send_float(RTD_a);
		  UART_send_float(RTD_b);
		  strcpy(uart_string, "");

	  }

	  if(strcmp(uart_string, "start:calibration\r") == 0)
	  {
		  potentiometer_calibration();
		  strcpy(uart_string, "");
	  }

	  if(strcmp(uart_string, "tracking:start\r") == 0)
	  {
		  //a = stabilize_v1(SERVO_MIN_ANGLE, 0.5, 0.1, a);
		  new_tick = DWT->CYCCNT;
		  tick=new_tick - startTick;
		  tick=tick/SystemCoreClock*1000;
		  a = stabilize_by_pid(&gst_pid, ONE_US*1000, tick);
		  UART_send_float(a);

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

void clear_uart_buffer(uint8_t counter, uint8_t len, uint8_t buffer[len]){
	for(uint8_t i=0; i<len; i++){
		buffer[i]='\0';
	}
}

//void get_r_a_function(int len, double angles[len], double resistances[len])
//{
//
//	for(int d = 0; d<len; d++)
//	  {
//		int a = servo_set_position(angles[d]);
//		  if(a != -1)
//		  {
//			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC1_DATA, 1);
//			  resistances[d]= (ADC1_DATA[0]*ADC1_K);
//
//			  HAL_Delay(10);
//		  }
//          HAL_Delay(1);
//	  }
//}


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

//	linear_regression(len,degs, ress, &DTR_a, &DTR_b);

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

//void void measure()
//{
//	  int i = 0;
//	  int mcount = 50;
//	  double stp = 0.5;
//	  char* inform = "Started filter test.\r\nmcount = 10";
//	  UART_Transmit_string_CR_LF(&huart1, inform, 10);
//	  for(double d = SERVO_MIN_ANGLE; d<=SERVO_MAX_ANGLE; d=d+stp)
//	  {
//		  servo_set_position(d);
//
//		  just_angles[i] = get_actual_angle();
//		  sma_angles[i] = SMA_angle(mcount);
//		  _wma_angles[i] =_WMA_angle(mcount);
//		  last_angles[i] = last_ited_angle(mcount);
//
//		  i++;
//	  }
//	  inform = "Ended filter test.\r\ngot = 4 arrays";
//	  UART_Transmit_string_CR_LF(&huart1, inform, 10);
//
////	  send_series_(ANGLE_RANGE*2, "sma_angles", sma_angles);
////	  send_series_(ANGLE_RANGE*2, "_wma_angles", _wma_angles);
////	  send_series_(ANGLE_RANGE*2, "last_angles", last_angles);
//}

void UART_Transmit_number_CR_LF(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	uint8_t end_code  = 13; //ASCII '\r'
	uint8_t enter_code  = 10; //ASCII '\r'

	HAL_UART_Transmit(huart, pData, Size, Timeout);
	HAL_Delay(10);
	HAL_UART_Transmit(huart, &enter_code, 1, Timeout);
	HAL_Delay(10);
	HAL_UART_Transmit(huart, &end_code, 1, Timeout);
	HAL_Delay(10);
}

void UART_Transmit_string_CR_LF(UART_HandleTypeDef *huart, char *pData, uint32_t Timeout)
{
	uint8_t end_code  = 13; //ASCII '\r'
	uint8_t enter_code  = 10; //ASCII '\r'

	HAL_UART_Transmit(huart, pData, strlen(pData), Timeout);
	HAL_Delay(10);
	HAL_UART_Transmit(huart, &end_code, 1, Timeout);
	HAL_Delay(10);
	HAL_UART_Transmit(huart, &enter_code, 1, Timeout);
	HAL_Delay(10);

}

void UART_send_float(float number)
{
	char* point = ".";
	char temp[100] = "";
	volatile int n = 0;

    volatile float reminder, quotient = 0.0;

	quotient = (int)(number);
	n = quotient;

	itoa(n, temp, 10);
	HAL_UART_Transmit(&huart1, temp, strlen(temp), 10);
	HAL_Delay(10);

	HAL_UART_Transmit(&huart1, point, 1, 10);
	HAL_Delay(10);

	reminder = fabs(number - quotient);
	reminder = (int)(reminder*1000);
	n = reminder;
	itoa(n, temp, 10);

	UART_Transmit_string_CR_LF(&huart1, temp, 10);
}

void send_series_(int len, float array[len])
{

	char* point = ".";
	char temp[100] = "";
	volatile int n = 0;

    volatile float reminder, quotient = 0.0;
    UART_Transmit_string_CR_LF(&huart1, "start", 10);

	for(int i = 0; i < len; i++)
	{

		quotient = (int)(array[i]);
        n = quotient;

        itoa(n, temp, 10);
		HAL_UART_Transmit(&huart1, temp, strlen(temp), 10);
		HAL_Delay(10);

  		HAL_UART_Transmit(&huart1, point, 1, 10);
		HAL_Delay(10);

		reminder = array[i] - quotient;
	    reminder = (int)(reminder*1000);
	    n = reminder;
        itoa(n, temp, 10);

		UART_Transmit_string_CR_LF(&huart1, temp, 10);
	}

    UART_Transmit_string_CR_LF(&huart1, "stop", 10);
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

//float anglex_from_acc()
//{
//	volatile float acc_y = IMU_get_acc(Y_AXIS);
//	HAL_Delay(1);
//	float anglex = 0.0;
//	acc_y=clamp(acc_y, -1.0, 1.0);
//	if(acc_y >=0)
//	{
//		anglex = 90-RAD_TO_DEG*(float)acos(acc_y);
//	}
//
//	else
//	{
//		anglex=RAD_TO_DEG*(float)acos(acc_y) - 90;
//	}
//
//	return anglex;
//}

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

float angle_from_gyro(int axis, float dt_us, float* previous_angle)
{
	volatile float gyro = IMU_get_gyro(axis);
	*previous_angle = *previous_angle + gyro*dt*1e-6;
    return *previous_angle;
}
float complementary_filter(float dt, int axis, float* angle_gyro, float* CFK)
{


	volatile float k = *CFK;
	volatile float angle_acc = 0;
    volatile float angle = 0;


	angle_acc = angle_from_acc(axis);
	*angle_gyro = angle_from_gyro(axis, dt, angle_gyro);

    angle= (*angle_gyro)*(1-k)+angle_acc*k;
    HAL_Delay(1);
	return angle;
}

float stabilize_v1(float hold_angle, float prev_angle, float precision, int time_step, int timer)
{
	  volatile float da;
	  volatile float imu_angle;
	  if((timer%time_step)!=0)
	  {
		  return prev_angle;
	  }
	  imu_angle = complementary_filter(time_step, Y_AXIS,
			                        &angle_gyro_y, &CFK);

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

float stabilize_by_pid(PIDtypedef *PID_struct, int time_step, int timer)
{
	  volatile float pid_error;
	  volatile float imu_angle;
	  volatile float set;
 //     HAL_Delay(1);
	  if((timer%time_step)!=0)
	  {
		  return PID_struct->out;
	  }
	  imu_angle = complementary_filter(time_step, Y_AXIS,
			                        &angle_gyro_y, &CFK);

	  if(imu_angle < IMU_MAX_ANGLE)
	  {
		  imu_angle = IMU_MAX_ANGLE;
	  }

	  if(imu_angle > IMU_MIN_ANGLE)
	  {
		  imu_angle = IMU_MIN_ANGLE;
	  }

	  imu_angle = imu_angle*ITS_B + ITS_A;
	  pid_error = fabs(PID_struct->hold - imu_angle);
	  PID(PID_struct, pid_error);
//	  if(PID_struct->out < SERVO_MIN_ANGLE)
//	  {
//		  set = SERVO_MAX_ANGLE-PID_struct->out;
//	  }
//	  else
//	  {
//		  set = PID_struct->out;
//	  }
	  set = SERVO_MAX_ANGLE-PID_struct->out;
//	  set = PID_struct->out;
	  servo_set_position(set);

	  return _WMA_angle(10);
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
