
#include "comm.h"
#include "fifo.h"
#include "stdlib.h"
#include "string.h"
#include "motor.h"

fifo	uartBuffer;
uint8_t uartTmpByte;
uint8_t uartState, flagEndFrame;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	FIFO_Push(UART_FIFO, uartTmpByte);
	if (uartTmpByte == FRAME_S)
		uartState = uartStartFrame;
	else if (uartTmpByte == FRAME_E)
	{
		uartState = uartEndFrame;
		flagEndFrame = 1;
	}
	HAL_UART_Receive_IT(&huart2, &uartTmpByte, 1);
}

void UART_Init()
{
	FIFO_Init(&uartBuffer, 100);
	HAL_UART_Receive_IT(COMM, &uartTmpByte, 1);
	uartState = uartInit;
}

void UART_CyclicTask()
{
	uint8_t tmpBuffer[100], i = 0;
	if (flagEndFrame)
	{
		if (FIFO_Peek(UART_FIFO) == FRAME_S)
			FIFO_Pop(UART_FIFO);
		else 
			return;
		
		while (FIFO_Peek(UART_FIFO) != '\n' && i < 100)
		{
			tmpBuffer[i] = FIFO_Pop(UART_FIFO);
			++i;
		}
		--i;
		if (i > 1)
			if (tmpBuffer[i] != '\n' || tmpBuffer[i - 1] != '\r')
				uartState = uartError;
		
		//if (!strncmp((const char *)tmpBuffer, "ED_OUT0_1", 9))
		// #function/motor/param
		
		char tmpA[1];
		tmpA[0] = tmpBuffer[0];
		uint8_t mot = atoi(tmpA);

		
		if (!strncmp(&tmpBuffer[2], "mDISABLE", 8))
		{
			MOTOR_Disable(M(mot));
		}
		else if (!strncmp(&tmpBuffer[2], "mENABLE", 7))
		{
			MOTOR_Enable(M(mot));
		}
		else if (!strncmp(&tmpBuffer[2], "mPWM", 4))
		{
			if (i >= 7)
			{
				char *tmp = &tmpBuffer[7];
				int16_t pwm = (int16_t)atoi(tmp);
				MOTOR_SetPWM(M(mot), pwm);
			}
		}
		else if (!strncmp(&tmpBuffer[2], "mTIME", 5))
		{
			if (i >= 8)
			{
				char *tmp = &tmpBuffer[8];
				int16_t pwm = (int16_t)atoi(tmp);
				
				int pos = strcspn(&tmpBuffer[8], "_");
				if (pos == 0 || pos >5)
					return;
				
				tmp = &tmpBuffer[8+pos+1];
				int16_t time = (int16_t)atoi(tmp);
				MOTOR_SetTime(M(mot), pwm, time);
			}
		}
		else if (!strncmp(&tmpBuffer[2], "mGo",3))
		{
			if (i >= 8)
			{
				char *tmp = &tmpBuffer[6];
				int16_t pwm = (int16_t)atoi(tmp);
				
				int pos = strcspn(&tmpBuffer[6], "_");
				if (pos == 0 || pos >5)
					return;
				
				tmp = &tmpBuffer[6+pos+1];
				int16_t time = (int16_t)atoi(tmp);
				
				if (mot == 1)
				{
					MOTOR_SetTime(M(1), pwm, time);
					MOTOR_SetTime(M(2), pwm, time);
					MOTOR_SetTime(M(3), pwm, time);
					MOTOR_SetTime(M(4), pwm, time);
				}
				if (mot == 4)
				{
					MOTOR_SetTime(M(8), pwm, time);
					MOTOR_SetTime(M(7), -pwm, time);
					MOTOR_SetTime(M(6), -pwm, time);
					MOTOR_SetTime(M(5), pwm, time);
				}
			}
		}

		
		
		
		uartState = uartWaitForData;
		flagEndFrame = 0;
		while (FIFO_Peek(UART_FIFO) != FRAME_S && FIFO_Size(UART_FIFO))
		{ 
			FIFO_Pop(UART_FIFO);
		}

		//		char tmpA[1];
		//		tmpA[0]= tmpBuffer[0];
		//		uint8_t mot = atoi(tmpA);
		//		tmpA[0] = tmpBuffer[1];
		//		uint8_t ster  = atoi(tmpA);
		//		switch (ster)
		//		{
		//		case motorDisable:
		//			MOTOR_Disable(M(mot));
		//			break;
		//		case motorEnable:
		//			MOTOR_Enable(M(mot));
		//			break;
		//		case motorPWMcontrol:
		//			{
		//				if (i >= 5)
		//				{
		//					char *tmp = &tmpBuffer[2];
		//					int16_t pwm = (int16_t)atoi(tmp);
		//					MOTOR_SetPWM(M(mot), pwm);
		//				}
		//				break;
		//			}
		//			
		//		case motorTimeControl:
		//			{
		//				if (i >= 5)
		//				{
		//					char *tmp = &tmpBuffer[2];
		//					int16_t pwm = (int16_t)atoi(tmp);
		//					char *tmp2 = &tmpBuffer[8];
		//					int16_t time = (int16_t)atoi(tmp2);
		//					MOTOR_SetTime(M(mot), pwm, (float)time);
		//				}
		//				break;
		//			}
		//		}
		//		uartState = uartWaitForData;
		//		flagEndFrame = 0;
		//		
		//		while (FIFO_Peek(UART_FIFO) != FRAME_S && FIFO_Size(UART_FIFO))
		//		{ 
		//			FIFO_Pop(UART_FIFO);
		//		}
	}
}


void subStr(char* input, char* output, uint8_t sepr, uint8_t number)
{
	
}