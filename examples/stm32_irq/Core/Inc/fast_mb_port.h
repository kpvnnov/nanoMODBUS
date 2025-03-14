/**
 ******************************************************************************
 * @file           : fast_mb_port.h
 * @brief          : Header for fast_mb_port.c file.
 *                   This file contains the common defines of the fastmodbus port stm32.
 ******************************************************************************
 nanoModbus stm32 fastmodbus port based on example irq
 Copyright (C) 2025 Peter Kostenko <kpvnnov@gmail.com> https://t.me/kpvnnov

 MIT License
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FASTMB_PORT_H__
#define __FASTMB_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif

void nano_RecieveMode(nmbs_t*);
void compute_timer(nmbs_t*);

typedef struct nmbs_arg_t {
	uint32_t FastModbus_Prescaler, Arbitrage_Period, Window_Period,
			Arbitrage_Periodx60, Window_Periodx60, Normal_Prescaler, Normal_Period;
	TIM_HandleTypeDef *htim; //указатель на таймер
	UART_HandleTypeDef *huart; //указатель на uart
	//void (*Timer_FastModbus)(nmbs_t*); //обработка прерываний таймера
	HAL_StatusTypeDef (*TIM_Base_Stop)(nmbs_t*); //остановка прерываний таймера
	HAL_StatusTypeDef (*TIM_Base_Start)(nmbs_t*); //запуск прерываний таймера
	HAL_StatusTypeDef (*UART_Receive_IT)(nmbs_t*); //запуска приема символа по прерыванию
	void (*fmb_RecieveMode)(nmbs_t*); //запуск приема символов в режиме fastmodbus
	void (*nano_RecieveMode)(nmbs_t*); //запуск приема символов в обычном режиме modbus
	//int32_t (*read)(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms,
    //                void* arg); /*!< Bytes read transport function pointer */
    uint8_t initialized; /*!< Reserved, workaround for older user code not calling nmbs_arg_create() */

} nmbs_arg_t;

inline void strobe_toggle() {
	if (get_debug_strobe()) {
		HAL_GPIO_TogglePin(STROBE_GPIO_Port, STROBE_Pin);
		MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"%ld strobe ",HAL_GetTick());
	}
}


#ifdef __cplusplus
}
#endif

#endif /* __FASTMB_PORT_H__ */
