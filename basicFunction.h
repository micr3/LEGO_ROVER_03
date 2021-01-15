#pragma once

#include "main.h"
#include "stm32f4xx_hal.h"

#define millis()	HAL_GetTick()

#define TIM_MOTOR_1_4	htim1
#define TIM_MOTOR_5_8	htim3

#define MOTOR_REFRESH	10	//ms

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

extern uint16_t shiftVal;

void SHIFT_Init();
void SHIFT_Set(uint8_t mNumber, uint8_t val);
void SHIFT_Update();

void PWM_Init();
void PWM_Set(uint8_t mNumber, uint16_t val);


void PWM_Init();
void PWM_Set(uint8_t mNumber, uint16_t val);


uint8_t timeIsUp(uint32_t old_time, uint32_t delay_time);
//uint32_t abs(int32_t x);