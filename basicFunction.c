
#include "basicFunction.h"

uint16_t shiftVal, shiftValFin;
uint8_t shiftOrder[8] = { 1, 2, 4, 3, 5, 6, 7, 8 };   //{1,2,4,3,8,7,6,5}
uint8_t shiftRever[8] = { 1, 1, 0, 0, 1, 1, 0, 0 };

void SHFIT_Init()
{
	HAL_GPIO_WritePin(SHIFT_LA_GPIO_Port, SHIFT_LA_Pin, GPIO_PIN_SET);
	shiftVal = 0x0000;
	SHIFT_Update();
}
void SHIFT_Set(uint8_t mNumber, uint8_t val)
{
	if (mNumber >= 1 && mNumber <= 8)
	{
		--mNumber;    	//motor number 0-7
		if(shiftRever[mNumber])
			val = ~val;
		val &= 0x03;     //mask
		mNumber = shiftOrder[mNumber] - 1;
		
		shiftVal &= ~(0x03 << (mNumber * 2));
		shiftVal |= (val << (mNumber * 2));
	}
}
void SHIFT_Update()
{
	shiftValFin = shiftVal;
	HAL_SPI_Transmit(&hspi1, &shiftValFin, 1, 5);
	HAL_GPIO_WritePin(SHIFT_LA_GPIO_Port, SHIFT_LA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SHIFT_LA_GPIO_Port, SHIFT_LA_Pin, GPIO_PIN_SET);
}

void PWM_Init()
{
	HAL_TIM_PWM_Start(&TIM_MOTOR_1_4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM_MOTOR_1_4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM_MOTOR_1_4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM_MOTOR_1_4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&TIM_MOTOR_5_8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM_MOTOR_5_8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM_MOTOR_5_8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM_MOTOR_5_8, TIM_CHANNEL_4);
	TIM_MOTOR_1_4.Instance->CCR1 = 0;
	TIM_MOTOR_1_4.Instance->CCR2 = 0;
	TIM_MOTOR_1_4.Instance->CCR3 = 0;
	TIM_MOTOR_1_4.Instance->CCR4 = 0;
	TIM_MOTOR_5_8.Instance->CCR1 = 0;
	TIM_MOTOR_5_8.Instance->CCR2 = 0;
	TIM_MOTOR_5_8.Instance->CCR3 = 0;
	TIM_MOTOR_5_8.Instance->CCR4 = 0;

}
void PWM_Set(uint8_t mNumber, uint16_t val)
{
	if (val > 1000)
		val = 1000;
	switch (mNumber)
	{
	case 1: TIM3->CCR1 = val; break;
	case 2: TIM3->CCR3 = val; break;
	case 3: TIM3->CCR2 = val; break;
	case 4: TIM3->CCR4 = val; break;
	case 5: TIM1->CCR4 = val; break;
	case 6: TIM1->CCR3 = val; break;
	case 7: TIM1->CCR2 = val; break;
	case 8: TIM1->CCR1 = val; break;
	}
}


uint8_t timeIsUp(uint32_t old_time, uint32_t delay_time)
{
	if (millis() - old_time > delay_time) return 1;
	else return 0;
}

//uint32_t abs(int32_t x)
//{
//	if (x >= 0) return x;
//	else return -x;
//}
