#pragma once

#include "main.h"
#include "stm32f4xx_hal.h"

#define COMM		&huart2
#define UART_FIFO	&uartBuffer

#define FRAME_S		'#'
#define FRAME_E		'\n'


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

enum uartState {
	uartNoInit      = 0,
	uartInit        = 1,
	uartWaitForData = 2,
	uartStartFrame  = 3,
	uartEndFrame    = 4,
	uartService		= 5,
	uartError       = 6,
};
enum commState
{
	commTestConn =0
};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
	
void UART_Init();
void UART_CyclicTask();

void subStr(char* input, char* output, uint8_t sepr, uint8_t number);