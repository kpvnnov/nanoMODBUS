/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
extern uint32_t Speed;
extern uint32_t FastModbus_Prescaler, Arbitrage_Period, Window_Period,
		Arbitrage_Periodx60, Window_Periodx60, Normal_Prescaler, Normal_Period;
/* USER CODE END 0 */

TIM_HandleTypeDef htim6;

/* TIM6 init function */
void MX_TIM6_Init(uint16_t timer_mode) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	/* USER CODE BEGIN TIM6_Init 1 */
	/* If baudrate > 19200 then we should use the fixed timer values
	 * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
	 For example, let's calculate the response timeout at a speed of 9600 bps
	 Time to transmit 1 bit at a given speed 9600 =
	 (1s/9600)*1000000=104.167  microsecond
	 104.167*3.5 * 11 (1 start + 8 date + 2 stop)= 4010,416 microsecond
	 Time to transmit 1 bit table
	 baud| microsecond   | для 48Мгц
	 1200 |   833.33      |
	 9600 |   104.166     |
	 19200 |    52.083     |
	 38400 |    26.042     |
	 57600 |    17.361     |
	 115200 |     8.681     |
	 начало арбитражного окна 36 бит
	 минимально 13 (арбитражное окно)

	 вычисляем две переменных
	 1)начало арбитража
	 x=SystemCoreClock/(Speed*10);
	 Prescaler=x-1;

	 Period при скорости 38400 и ниже = используется k=36
	 y(второй делитель)=SystemCoreClock*k/(Speed*x)
	 если скорость выше 38400
	 то расчёт k=8*speed/100000+1
	 Period=y-1

	 2)длина арбитражного окна

	 используется коэффициент k=12 + ROUNDUP(50 мкс/длительность одного бода в мкс)
	 k=12+5*speed/100000+1
	 Prescaler и Period рассчитываются аналогично арбитражному

	 x(первый делитель)=SystemCoreClock/(Speed*10);
	 Prescaler=x-1;
	 y(второй делитель )=SystemCoreClock*k/(Speed*x)
	 Period=y-1
	 */
	/*
	 * Для команды сканирования (0x60) действуют устаревшие правила арбитража,
	 * в которых длительность окна составляет 20 бит при текущем baud rate,
	 * а начало арбитража - через 44 бита после последнего принятого байта запроса - 3.5 байта.
	 */
	switch (timer_mode) {
	case 0:
		//normal 3.5 mode timer
		htim6.Init.Prescaler = Normal_Prescaler;
		htim6.Init.Period = Normal_Period;

		break;
	case 1:
		// begin arbitrage max(3 symbols, (12 bits + 800us)) для новой 0x46
		htim6.Init.Prescaler = FastModbus_Prescaler;
		htim6.Init.Period = Arbitrage_Period;

		break;
	case 2:
		// arbitrage windows для новой 0x46
		htim6.Init.Prescaler = FastModbus_Prescaler;
		htim6.Init.Period = Window_Period;
		break;
	case 3:
		// begin arbitrage max(3 symbols, (12 bits + 800us)) для старой 0x60
		htim6.Init.Prescaler = FastModbus_Prescaler;
		htim6.Init.Period = Arbitrage_Periodx60;

		break;
	case 4:
		// arbitrage windows для старой 0x60
		htim6.Init.Prescaler = FastModbus_Prescaler;
		htim6.Init.Period = Window_Periodx60;
		break;
	default:
		Error_Handler();
	}

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle) {

	if (tim_baseHandle->Instance == TIM6) {
		/* USER CODE BEGIN TIM6_MspInit 0 */

		/* USER CODE END TIM6_MspInit 0 */
		/* TIM6 clock enable */
		__HAL_RCC_TIM6_CLK_ENABLE();

		/* TIM6 interrupt Init */
		HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM6_IRQn);
		/* USER CODE BEGIN TIM6_MspInit 1 */

		/* USER CODE END TIM6_MspInit 1 */
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle) {

	if (tim_baseHandle->Instance == TIM6) {
		/* USER CODE BEGIN TIM6_MspDeInit 0 */

		/* USER CODE END TIM6_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM6_CLK_DISABLE();

		/* TIM6 interrupt Deinit */
		HAL_NVIC_DisableIRQ(TIM6_IRQn);
		/* USER CODE BEGIN TIM6_MspDeInit 1 */

		/* USER CODE END TIM6_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
