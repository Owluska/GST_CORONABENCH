/*
 * USB_comunication.h
 *
 *  Created on: 8 may 2020 ã.
 *      Author: Kagirina K.A.
 */

#ifndef USB_COMUNICATION_H_
#define USB_COMUNICATION_H_


#include "main.h"
void clear_uart_buffer(uint8_t counter, uint8_t len,
		               uint8_t buffer[len]);
void UART_Transmit_string(UART_HandleTypeDef *huart,
		                    char *pData, uint32_t Timeout);
void UART_Transmit_number_CR_LF(UART_HandleTypeDef *huart,
		  uint8_t *pData, uint16_t Size, uint32_t Timeout);
void UART_Transmit_string_CR_LF(UART_HandleTypeDef *huart,
		                    char *pData, uint32_t Timeout);
void UART_send_float(UART_HandleTypeDef *huart, float number, uint32_t Timeout);
void send_series_(int len, float array[len]);



#endif /* USB_COMUNICATION_H_ */
