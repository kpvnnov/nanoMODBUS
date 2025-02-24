/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

extern uint32_t Speed;
/* USER CODE END 0 */

UART_HandleTypeDef UartDebug;
UART_HandleTypeDef modbusUart;

/* USART1 init function */

void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	UartDebug.Instance = USART1;
	UartDebug.Init.BaudRate = 115200;
	UartDebug.Init.WordLength = UART_WORDLENGTH_8B;
	UartDebug.Init.StopBits = UART_STOPBITS_1;
	UartDebug.Init.Parity = UART_PARITY_NONE;
	UartDebug.Init.Mode = UART_MODE_TX_RX;
	UartDebug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartDebug.Init.OverSampling = UART_OVERSAMPLING_16;
	UartDebug.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	UartDebug.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&UartDebug) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_ModbusUart_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	modbusUart.Instance = USART2;
	modbusUart.Init.BaudRate = get_baudrate() * 100UL;
	modbusUart.Init.WordLength = UART_WORDLENGTH_8B;
	modbusUart.Init.StopBits = UART_STOPBITS_2;
	modbusUart.Init.Parity = UART_PARITY_NONE;
	modbusUart.Init.Mode = UART_MODE_TX_RX;
	modbusUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	modbusUart.Init.OverSampling = UART_OVERSAMPLING_16;
	modbusUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	modbusUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT
			| UART_ADVFEATURE_DMADISABLEONERROR_INIT;
	modbusUart.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	modbusUart.AdvancedInit.DMADisableonRxError =
	UART_ADVFEATURE_DMA_DISABLEONRXERROR;
	if (HAL_UART_Init(&modbusUart) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (huart->Instance == USART1) {
		/* USER CODE BEGIN USART1_MspInit 0 */

		/* USER CODE END USART1_MspInit 0 */
		/* USART1 clock enable */
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		USARTx_TX_GPIO_CLK_ENABLE();
		//USARTx_RX_GPIO_CLK_ENABLE();
		__HAL_RCC_USART1_CLK_ENABLE();

		/* Enable USARTx clock */
		USARTx_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin = USARTx_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		//GPIO_InitStruct.Speed     = GPIO_SPEED_FAST; // also kown as GPIO_SPEED_FREQ_HIGH
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // also kown as GPIO_SPEED_FREQ_HIGH
		GPIO_InitStruct.Alternate = USARTx_TX_AF;

		HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration          */
		//GPIO_InitStruct.Pin = USARTx_RX_PIN;
		//GPIO_InitStruct.Alternate = USARTx_RX_AF;
		//HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
		/* USART1 interrupt Init */
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		/* USER CODE BEGIN USART1_MspInit 1 */

		/* USER CODE END USART1_MspInit 1 */
	} else if (huart->Instance == USART2) {
		/* USER CODE BEGIN USART2_MspInit 0 */

		/* USER CODE END USART2_MspInit 0 */
		/* USART2 clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART2 GPIO Configuration
		 PA2     ------> USART2_TX
		 PA3     ------> USART2_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART2 interrupt Init */
		HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		/* USER CODE BEGIN USART2_MspInit 1 */

		/* USER CODE END USART2_MspInit 1 */
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
		/* USER CODE BEGIN USART1_MspDeInit 0 */

		/* USER CODE END USART1_MspDeInit 0 */
		/* Peripheral clock disable */
		USARTx_CLK_DISABLE();

		/* Peripheral clock disable */
		USARTx_TX_GPIO_CLK_DISABLE();
		//USARTx_RX_GPIO_CLK_DISABLE();

		HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
		// HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

		/* USART1 interrupt Deinit */
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		/* USER CODE BEGIN USART1_MspDeInit 1 */

		/* USER CODE END USART1_MspDeInit 1 */
	} else if (huart->Instance == USART2) {
		/* USER CODE BEGIN USART2_MspDeInit 0 */

		/* USER CODE END USART2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART2_CLK_DISABLE();

		/**USART2 GPIO Configuration
		 PA2     ------> USART2_TX
		 PA3     ------> USART2_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

		/* USART2 interrupt Deinit */
		HAL_NVIC_DisableIRQ(USART2_IRQn);
		/* USER CODE BEGIN USART2_MspDeInit 1 */

		/* USER CODE END USART2_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
