/**
 ******************************************************************************
 * @file           : fast_mb_port.c
 * @brief          : port stm32 for fast modbus extension modbus protocol
 ******************************************************************************
 base on nanoModbus stm32 example irq
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
#include "main.h"
#include "usart.h"
#include "nanomodbus.h"
#include "fast_mb.h"
#include "fast_mb_port.h"


/*
 #define COUNTER_TIM_FLAG 200
 inline void clear_tim_flag(nmbs_t *nmbs) {
 volatile uint16_t counter = COUNTER_TIM_FLAG;
 nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);
 while (!HAL_IS_BIT_SET(params->htim->Instance->SR, TIM_FLAG_UPDATE)
 && (counter--)) {
 if (counter == 0) {
 MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "\n!!BUG clear_tim_flag!!\n ");
 critical_stop();
 }
 }
 if (counter != COUNTER_TIM_FLAG)
 //MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG, "\n!!!clear_tim_flag %d ",counter);
 MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\n!!!clear_tim_flag %d ",counter);
 CLEAR_BIT(params->htim->Instance->SR, TIM_FLAG_UPDATE);
 }
 */
extern volatile bool must_reload_rs485;
extern volatile bool packet_sended; //была ли в текущем цикле передача?



//чтобы не терять время на математические операции коэффициенты делителя рассчитать заранее
/* вычисляем две переменных
 1)начало арбитража
 x=SystemCoreClock/(Speed*10);
 Prescaler=x-1;

 Period при скорости 38400 и ниже = используется k=36
 y(второй делитель)=SystemCoreClock*k/(Speed*x)
 если скорость выше 38400
 то расчёт k=8*speed/10000+1
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

/*
 первая проба измерения
 Speed:9600
 FastModbus_Prescaler:499 Arbitrage_Period:359 Window_Period:129
 Normal_Prescaler:2399 Normal_Period:80

 0x60
 MR6 4.6481-  ms ожидание арбитража  2.08376 ms арбитражное окно
 6DO  ms ожидание арбитража  mks арбитражное окно

 x46
 MR6 3.719878(-одно арб окно) ms ожидание арбитража 1.3537  ms арбитражное окно
 6DO  ms ожидание арбитража  mks арбитражное окно

 добавил два вида интервалов, вторая проверка
 Speed:9600
 FastModbus_Prescaler:499
 Arbitrage_Period:  359     Window_Period:  129
 Old Arbitrage_Period:  439 old Window_Period:  199
 Normal_Prescaler:2399 Normal_Period:80
 x46
 MR6 3.719878 ms (35.7 бита?) ожидание арбитража 1.3537 (ровно 13!)  ms арбитражное окно
 x60
 MR6 4.648359  ms (44 бита?) ожидание арбитража 2.08677 ms (ровно 20!!) арбитражное окно

 0x46
 6DO 5.981 ms ожидание арбитража  1.353758 (ровно 13!) mks арбитражное окно
 0x60
 6DO 4,540645 ms (44 бита?) ожидание арбитража 2.08677 ms (ровно 20!!) арбитражное окно

 */
/*
 Speed:19200
 FastModbus_Prescaler:249 Arbitrage_Period:359 Window_Period:129
 Normal_Prescaler:2399 Normal_Period:40
 MR6 1.6569 ms ожидание арбитража 1.11798 ms арбитражное окно
 6DO 2.316 ms ожидание арбитража 677 mks арбитражное окно
 */
/*
 Speed:38400
 FastModbus_Prescaler:124 Arbitrage_Period:359 Window_Period:139
 Normal_Prescaler:2399 Normal_Period:34
 MR6  1.3515 ms ожидание арбитража 1.0034 ms арбитражное окно
 6DO 1.218 ms ожидание арбитража  364 mks арбитражное окно
 */
/*
 Speed:115200
 FastModbus_Prescaler:40 Arbitrage_Period:100 Window_Period:181
 Normal_Prescaler:2399 Normal_Period:34
 MR6 1.01 ms ожидание арбитража 172.967 mks арбитражное окно

 */
void compute_timer(nmbs_t *nmbs) {

	uint32_t x, y;
	uint32_t k;
	uint32_t baudrate = get_baudrate() * 100UL;
	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);

	params->Normal_Prescaler = SystemCoreClock / (1000000UL / 50UL) - 1;
	if (baudrate > 19200) {
		params->Normal_Period = (1750 / 50) - 1;
	} else {
		//оптимизирую умножение 3.5*11=38,5
		//Normal_Period = ((7UL * 1000000UL * 11UL / (50UL * 2UL)) / baudrate); //-1 absent for rounding up
		params->Normal_Period = ((40UL * 1000000UL / 50UL) / baudrate); //-1 absent for rounding up

	}

	x = SystemCoreClock / (uint32_t) (baudrate * 10UL);
	params->FastModbus_Prescaler = x - 1;
//вычисляем начало арбитража для новой команды 0x46
	if (baudrate <= 38400) {
		k = 36;

	} else {
		k = 8UL * baudrate / 10000UL + 1;
	}
	y = SystemCoreClock * k / (uint32_t) (baudrate * x);
	params->Arbitrage_Period = y - 1;

	//вычисляем начало арбитража для старой команды 0x60
	k = 44;
	y = SystemCoreClock * k / (uint32_t) (baudrate * x);
	params->Arbitrage_Periodx60 = y - 1;

	//длина арбитражного окна для новой команды 0x46
	k = 12UL + 5UL * baudrate / 100000UL + 1;
	y = SystemCoreClock * k / (uint32_t) (baudrate * x);
	params->Window_Period = y - 1;

	//длина арбитражного окна для старой команды 0x60
	k = 20;
	y = SystemCoreClock * k / (uint32_t) (baudrate * x);
	params->Window_Periodx60 = y - 1;
	MP_FMB_DEBUG_PRINT(DEBUG_INFO,"Speed:%d Address:%d\n", get_baudrate(), get_modbusaddress());
	MP_FMB_DEBUG_PRINT(DEBUG_INFO,"FastModbus_Prescaler:%ld\n", params->FastModbus_Prescaler);
	MP_FMB_DEBUG_PRINT(DEBUG_INFO,"    Arbitrage_Period:%5ld     Window_Period:%5ld\n",
			params->Arbitrage_Period, params->Window_Period);
	MP_FMB_DEBUG_PRINT(DEBUG_INFO,"Old Arbitrage_Period:%5ld old Window_Period:%5ld\n",
			params->Arbitrage_Periodx60, params->Window_Periodx60);
	MP_FMB_DEBUG_PRINT(DEBUG_INFO,"Normal_Prescaler:%ld Normal_Period:%ld\n", params->Normal_Prescaler,
			params->Normal_Period);

}

//HAL_StatusTypeDef UART_Receive(nmbs_t *nmbs){
//	return HAL_UART_Receive_IT(((nmbs_arg_t*) nmbs->platform.arg)->huart, &nmbs->msg.buf[nmbs->msg.buf_rec], 1);
//}

int32_t write_serial(const uint8_t *buf, uint16_t count,
		int32_t byte_timeout_ms, void *arg) {
	//nmbs_t *params = (nmbs_t*) arg;
	nmbs_arg_t *params = (nmbs_arg_t*) arg;
	nmbs_t *nmbs = params->my_nmsb;


	packet_sended = true;
	HAL_StatusTypeDef res;
	//перенесли отмену приёма в таймер
	__HAL_ENTER_CRITICAL_SECTION();
	if (HAL_UART_STATE_BUSY_RX == params->huart->RxState) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"!!!write_serial wrong HAL_UART_STATE_BUSY_RX\n");
		//res = HAL_UART_AbortReceive(&modbusUart);
		res = params->UART_AbortReceive(nmbs);
		if (HAL_OK != res) {
			MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"write_serial UART_AbortReceive error %d\n", res);
			critical_stop();
		}
	}
	if (params->huart->gState == HAL_UART_STATE_READY) {
		SetRS485Transmit();
		//res = HAL_UART_Transmit_IT(&modbusUart, buf, count);
		res = params->UART_Transmit(nmbs, buf, count);
		if (res != HAL_OK) {
			MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"UART_Transmit error %d\n", res);
			critical_stop();
		}
	} else {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"error state:%ld write_serial UART_Transmit \n", params->huart->gState);
	}
	__HAL_EXIT_CRITICAL_SECTION();
	return count;
}

void fastmodbus_RecieveMode(nmbs_t *nmbs) {
	HAL_StatusTypeDef res;
	strobe_toggle();
	__HAL_ENTER_CRITICAL_SECTION();
	//nmbs->msg.buf_rec = 0; //нет смысла забивать этими данными буфер, поэтому всегда обнуляем
	msg_rec_reset(nmbs); //нет смысла забивать этими данными буфер, поэтому всегда обнуляем
	SetRS485Receive();

	//Receive of data in IRQ Mode

	res = ((nmbs_arg_t*) nmbs->platform.arg)->UART_Receive(nmbs);
	__HAL_EXIT_CRITICAL_SECTION();
	//if (HAL_UART_Receive_IT(&modbusUart, nmbs.msg.buf, 1) != HAL_OK) {
	if (HAL_OK != res) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "HAL_UART_Receive_IT error:%d\n",res);
		critical_stop();
	}
	strobe_toggle();
}
void nano_RecieveMode(nmbs_t *nmbs) {
	HAL_StatusTypeDef res;
	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);
	strobe_toggle();
	__HAL_ENTER_CRITICAL_SECTION();

	/* Generate an update event to reload the Prescaler
	 and the repetition counter (only for advanced timer) value immediately */
	params->htim->Instance->EGR = TIM_EGR_UG;
	SetRS485Receive();
	packet_sended = false;
	if (must_reload_rs485) { //надо перезапустить RS-485 с новыми коммуникационными параметрами
		must_reload_rs485 = false;
		HAL_UART_DeInit(params->huart); //&modbusUart
		ModbusUart_Init(); //@todo надо тоже отвязать
	}
	msg_rec_reset(nmbs);
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\n%ld nano_RecMode buf_rec:%d\n",
			HAL_GetTick(), nmbs->msg.buf_rec);
	//Receive of data in IRQ Mode
	//res = HAL_UART_Receive_IT(params->huart, nmbs->msg.buf, 1); //&modbusUart
	res = params->UART_Receive(nmbs);
	if (res != HAL_OK) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "HAL_UART_Receive_IT error %d\n",res);
		critical_stop();
	}
	clear_tim_flag(nmbs);
	res = params->TIM_Start(nmbs);	//  HAL_TIM_Base_Start_IT(&TimerFastMB);
	__HAL_EXIT_CRITICAL_SECTION();

	if (res != HAL_OK) {
		/* Starting Error */
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "HAL_TIM_Base_Start_IT error %d\n",res);
		critical_stop();
	}

}

HAL_StatusTypeDef Start_Timer(nmbs_t *nmbs) {
	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);
	return HAL_TIM_Base_Start_IT(params->htim);
}

HAL_StatusTypeDef Stop_Timer(nmbs_t *nmbs) {
	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);
	return HAL_TIM_Base_Stop_IT(params->htim);
}
HAL_StatusTypeDef Receive_Serial(nmbs_t *nmbs) {
	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);

	return HAL_UART_Receive_IT(params->huart, &nmbs->msg.buf[nmbs->msg.buf_rec],
			1);
}
HAL_StatusTypeDef Transmit_Serial(nmbs_t *nmbs , uint8_t *pData, uint16_t Size) {
//	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs_arg);
	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);
	return HAL_UART_Transmit_IT(params->huart, pData, Size);
}

HAL_StatusTypeDef Abort_Serial(nmbs_t *nmbs) {
	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);

	return HAL_UART_AbortReceive(params->huart);
}
HAL_StatusTypeDef TIM_ReStart(uint8_t timer_mode, nmbs_t *nmbs) {
	nmbs_arg_t *params = ((nmbs_arg_t*) nmbs->platform.arg);
	HAL_StatusTypeDef res = HAL_TIM_Base_DeInit(params->htim);
	if (HAL_OK != res)
		return res;
	return MX_TIM_FastMB_Init(timer_mode, nmbs);//инициализируем таймер на указанный режим

}
