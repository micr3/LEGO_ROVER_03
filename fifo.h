#pragma once

#include "stm32f4xx_hal.h"

typedef struct
{
	uint8_t *buff;
	uint16_t in, out, size, max_size; 
} fifo;

void FIFO_Init(fifo *f, uint16_t size);
void FIFO_Push(fifo *f, uint8_t val);
void FIFO_Clear(fifo *f);

uint8_t FIFO_Pop(fifo *f);
uint8_t FIFO_Peek(fifo *f);
uint8_t FIFO_Size(fifo *f);