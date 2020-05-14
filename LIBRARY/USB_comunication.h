/*
 * USB_comunication.h
 *
 *  Created on: 8 may 2020 ã.
 *      Author: Kagirina K.A.
 */

#ifndef USB_COMUNICATION_H_
#define USB_COMUNICATION_H_


#include "main.h"
/**
 * @brief Used to clear uart bytes data buffer after end symbol
 * were red
 * @param counter: number of recieved bytes
 * @param len: buffer array length
 * @param buffer[len]: recieved bytes buffer
 */
void clear_uart_buffer( uint8_t len, uint8_t buffer[len]);
/**
 * @brief Transmit one by one chars from pointer to
 * char string through uart
 * @param *huart: pointer to @ref UART_HandleTypeDef structure
 * @param *pData: char string
 * @param Timeout: timeout for transmit operation
 */
void UART_Transmit_string(UART_HandleTypeDef *huart,
		                    char *pData, uint32_t Timeout);
/**
 * @brief Transmit byte with carriage return and line feed
 * @param *huart: pointer to @ref UART_HandleTypeDef structure
 * @param *pData: byte
 * @param Timeout: timeout for transmit operation
 */
void UART_Transmit_number_CR_LF(UART_HandleTypeDef *huart,
		                 uint8_t *pData, uint32_t Timeout);
/**
 * @brief Transmit string with carriage return and line feed
 * @param *huart: pointer to @ref UART_HandleTypeDef structure
 * @param *pData: char string
 * @param Timeout: timeout for transmit operation
 */
void UART_Transmit_string_CR_LF(UART_HandleTypeDef *huart,
		                    char *pData, uint32_t Timeout);
/**
 * @brief Transmit float number with carriage return and line feed
 * @param *huart: pointer to @ref UART_HandleTypeDef structure
 * @param number: char string
 * @param Timeout: timeout for transmit operation
 */
void UART_send_float(UART_HandleTypeDef *huart, float number, uint32_t Timeout);
/**
 * @brief Transmit series of float numbers divided by carriage return and line feed
 * @param *huart: pointer to @ref UART_HandleTypeDef structure
 * @param len: array length
 * @param array[len]: array to transmit
 * @param Timeout: timeout for transmit operation
 */
void send_series_(UART_HandleTypeDef *huart, int len,
		             float array[len], uint32_t Timeout);



#endif /* USB_COMUNICATION_H_ */
