#pragma once

#include "fifo.h"
#include "stdlib.h"

void FIFO_Init(fifo *f, uint16_t size)
{
	f->in = f->out = f->size = 0;
	f->max_size = size;
	f->buff = (uint8_t*) calloc(size, sizeof(uint8_t));
}

void FIFO_Push(fifo *f, uint8_t val)
{
	if (f->size < f->max_size)
	{
		f->buff[f->in] = val;
		if (f->in < f->max_size - 1)
			++f->in;
		else f->in = 0;

		++f->size;
	}
}
void FIFO_Clear(fifo *f)
{
	while (f->size)
		FIFO_Pop(f);
}

uint8_t FIFO_Pop(fifo *f)
{
	if (f->size > 0)
	{
		--f->size;
		if (f->out < f->max_size - 1)
			return f->buff[f->out++];
		else
		{
			f->out = 0;
			return f->buff[f->max_size - 1];
		}
	}
	else 
		return 0;
}

uint8_t FIFO_Peek(fifo *f)
{
	return f->size ? f->buff[f->out] : 0;
}

uint8_t FIFO_Size(fifo *f)
{
	return f->size;
}