/*
 * USB_comunication.c
 *
 *  Created on: 8 мая 2020 г.
 *      Author: Kagirina K.A.
 */

#include "USB_comunication.h"

//#include "main.h"
#include "usart.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

void clear_uart_buffer(uint8_t counter, uint8_t len,
		               uint8_t buffer[len]){
	for(uint8_t i=0; i<len; i++){
		buffer[i]='\0';
	}
}

void UART_Transmit_number_CR_LF(UART_HandleTypeDef *huart,
		  uint8_t *pData, uint16_t Size, uint32_t Timeout)
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

void UART_Transmit_string(UART_HandleTypeDef *huart,
		                    char *pData, uint32_t Timeout)
{
	uint8_t data;
	while(*pData != '\0')
	{

	    data=(uint8_t)(*pData);
		HAL_UART_Transmit(huart, &data, 1, Timeout);
	    pData++;
	}
}


void UART_Transmit_string_CR_LF(UART_HandleTypeDef *huart,
		                    char *pData, uint32_t Timeout)
{
	uint8_t end_code  = 13; //ASCII '\r'
	uint8_t enter_code  = 10; //ASCII '\r'

	UART_Transmit_string(huart, pData, Timeout);
//	HAL_UART_Transmit(huart, pData, strlen(pData), Timeout);
//	HAL_Delay(10);
//
//	HAL_UART_Transmit(huart, &end_code, 1, Timeout);
//	HAL_Delay(10);
//	HAL_UART_Transmit(huart, &enter_code, 1, Timeout);
//	HAL_Delay(10);
	UART_Transmit_string(huart, &end_code, Timeout);
	UART_Transmit_string(huart, &enter_code, Timeout);

}

void UART_send_float(UART_HandleTypeDef *huart, float number, uint32_t Timeout)
{
	char* point = ".";
	char temp[100] = "";
	volatile int n = 0;

    volatile float reminder, quotient = 0.0;

	quotient = (int)(number);
	n = quotient;

	itoa(n, temp, 10);

	UART_Transmit_string(huart, temp, Timeout);
//	HAL_UART_Transmit(huart, temp, strlen(temp), Timeout);
//	HAL_Delay(10);

//	HAL_UART_Transmit(huart, point, 1, Timeout);
//	HAL_Delay(10);
	UART_Transmit_string(huart, point, Timeout);

	reminder = fabs(number - quotient);
	reminder = (int)(reminder*1000);
	n = reminder;
	itoa(n, temp, 10);

	UART_Transmit_string_CR_LF(huart, temp, Timeout);
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
