/**
 ******************************************************************************
 * @file           : fast_mb.c
 * @brief          : fast modbus extension for modbus protocol
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
/* Includes ------------------------------------------------------------------*/
//#include <stdbool.h>
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "nanomodbus.h"
#include "fast_mb.h"

#ifdef NMBS_DEBUG

inline void strobe_toggle() {
	if (get_debug_strobe()) {
		HAL_GPIO_TogglePin(STROBE_GPIO_Port, STROBE_Pin);
		MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"%ld strobe ",HAL_GetTick());
	}
}

extern volatile bool debug_uart_run;
void flush_debug();

#else
#define strobe_toggle() (void) (0)
#endif

nmbs_error answer_scan(nmbs_t *nmbs);

uint32_t FastModbus_Prescaler, Arbitrage_Period, Window_Period,
		Arbitrage_Periodx60, Window_Periodx60, Arbitrage_Periodx60,
		Window_Periodx60, Normal_Prescaler, Normal_Period;

modbus_mode fast_mb_mode; //что делаем при приемё команды fastmodbus
uint16_t arbitrage_window; //текущий номер арбитражного окна
uint32_t arbitrage_word; //32 битное арбитражное слово
//uint32_t fastmodbus_address = 4265607340; //(dec) или 0xFE4000AC 32 битный уникальный fastmodbus адрес устройства

bool i_am_not_scaned = false;
bool arbitrage_loss; //признак проигранного арбитража

extern nmbs_t nmbs;
extern volatile uint8_t packet_sended;

inline void clear_tim_flag() {
	volatile uint8_t counter = 100;
	while (!HAL_IS_BIT_SET(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE) && counter--) {
		if (counter == 0) {
			MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "\n!!BUG clear_tim_flag!!\n ");
			critical_stop();
		}
	}
	if (counter != 100)
		MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG, "\n!!!clear_tim_flag %d ",
				counter);
	CLEAR_BIT(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE);
}
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
void compute_timer() {

	uint32_t x, y;
	uint32_t k;
	uint32_t baudrate = get_baudrate() * 100UL;

	Normal_Prescaler = SystemCoreClock / (1000000UL / 50UL) - 1;
	if (baudrate > 19200) {
		Normal_Period = (1750 / 50) - 1;
	} else {
		Normal_Period = ((7UL * 1000000UL * 11UL / (50UL * 2UL)) / baudrate); //-1 absent for rounding up
	}

	x = SystemCoreClock / (uint32_t) (baudrate * 10UL);
	FastModbus_Prescaler = x - 1;
//вычисляем начало арбитража для новой команды 0x46
	if (baudrate <= 38400) {
		k = 36;

	} else {
		k = 8UL * baudrate / 10000UL + 1;
	}
	y = SystemCoreClock * k / (uint32_t) (baudrate * x);
	Arbitrage_Period = y - 1;

	//вычисляем начало арбитража для старой команды 0x60
	k = 44;
	y = SystemCoreClock * k / (uint32_t) (baudrate * x);
	Arbitrage_Periodx60 = y - 1;

	//длина арбитражного окна для новой команды 0x46
	k = 12UL + 5UL * baudrate / 100000UL + 1;
	y = SystemCoreClock * k / (uint32_t) (baudrate * x);
	Window_Period = y - 1;

	//длина арбитражного окна для старой команды 0x60
	k = 20;
	y = SystemCoreClock * k / (uint32_t) (baudrate * x);
	Window_Periodx60 = y - 1;

}
void make_arbitrage_data(nmbs_t *nmbs) {
	if (i_am_not_scaned) {
		arbitrage_word = (0b0110 << 28)
				| (nmbs->msg.fastmodbus_address & 0x0FFFFFFF);
	} else {
		//а у отсканированных — с низким (0b1111)
		arbitrage_word = (0b1111 << 28)
				| (nmbs->msg.fastmodbus_address & 0x0FFFFFFF);
	}
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\narb_word:%08lx\n", arbitrage_word);
}
fast_mb_command check_fast_modbus(nmbs_t *nmbs, uint8_t length) {
	if (length && nmbs->msg.buf_rec != length)
		return fast_mb_none;
	MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG, "got %d bytes\n", length);
	if (nmbs->msg.buf[0] != 0xFD)
		return fast_mb_none;
	if (nmbs->msg.buf[1] != 0x60 && nmbs->msg.buf[1] != 0x46)
		return fast_mb_none;
	MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG, "got broadcast\n");
	switch (nmbs->msg.buf[2]) {
	case 0x01: //Функция начала сканирования - 0x01
		//MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"fb func 1\n");
		if (nmbs->msg.buf[1] == 0x46 && nmbs->msg.buf[3] == 0x13
				&& nmbs->msg.buf[4] == 0x90) { //проверка crc
			nmbs->msg.old_arbitrage = false;
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "begin 0x46 fmb scan\n");
			return fast_mb_begin_scan;
		}
		if (ASK_OLD_FASTMODBUS && nmbs->msg.buf[1] == 0x60
				&& nmbs->msg.buf[3] == 0x09 && nmbs->msg.buf[4] == 0xF0) { //проверка crc
			nmbs->msg.old_arbitrage = true;
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "begin 0x60 fmb scan\n");
			return fast_mb_begin_scan;
		}
		break;
	case 0x02: // Функция продолжения сканирования - 0x02
		MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\nfb func 2\n");

		if (nmbs->msg.buf[1] == 0x46 && nmbs->msg.buf[3] == 0x53
				&& nmbs->msg.buf[4] == 0x91) { //проверка crc
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "next 0x46 fmb scan\n");
			return fast_mb_next_scan;
		}
		if (ASK_OLD_FASTMODBUS && nmbs->msg.buf[1] == 0x60
				&& nmbs->msg.buf[3] == 0x49 && nmbs->msg.buf[4] == 0xF1) { //проверка crc
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "next 0x46 fmb scan\n");
			return fast_mb_next_scan;
		}

		break;
	case 0x08: // субкоманда эмуляции стандартных запросов
		if (nmbs->msg.buf_rec < 7) { //если приняли недостаточно байтов, чтобы проверить серийный номер
			return fast_mb_none;
		}
		//проверяем наш ли это серийный номер
		if (((nmbs->msg.buf[3] << 24) & (nmbs->msg.buf[4] << 16)
				& (nmbs->msg.buf[5] << 8) & (nmbs->msg.buf[6]))
				!= nmbs->msg.fastmodbus_address)
			return fast_mb_none;
		return fast_mb_emulate;
		//break;
	default:
		MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "normal fb\n");
		return fast_mb_none;
	}
	return fast_mb_none;
}
void fastmodbus_RecieveMode(void) {
	strobe_toggle();
	SetRS485Receive();

//Receive of data in IRQ Mode
	if (HAL_UART_Receive_IT(&huart2, nmbs.msg.buf, 1) != HAL_OK) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "HAL_UART_Receive_IT error\n");
		critical_stop();
	}
	strobe_toggle();
}
void nano_RecieveMode(void) {
	strobe_toggle();
	/* Generate an update event to reload the Prescaler
	 and the repetition counter (only for advanced timer) value immediately */
	TimerFastMB.Instance->EGR = TIM_EGR_UG;
	SetRS485Receive();
	packet_sended = false;
    if (must_reload_rs485) { //надо перезапустить RS-485 с новыми коммуникационными параметрами
	     must_reload_rs485 = 0;
	     HAL_UART_DeInit(&modbusUart);
	     MX_ModbusUart_UART_Init();
	}
	msg_rec_reset(&nmbs);
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\n%ld nano_RecieveMode buf_rec:%d\n",
			HAL_GetTick(), nmbs.msg.buf_rec);
//Receive of data in IRQ Mode

	if (HAL_UART_Receive_IT(&modbusUart, nmbs.msg.buf, 1) != HAL_OK) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "HAL_UART_Receive_IT error\n");
		critical_stop();
	}
	clear_tim_flag();

	if (HAL_TIM_Base_Start_IT(&TimerFastMB) != HAL_OK) {
		/* Starting Error */
		critical_stop();
	}

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &modbusUart) {
		switch (fast_mb_mode) {
		case mb_none:	//продолжаем обычный приём
			//after end of transmit go in receive mode
			nano_RecieveMode();
			break;
		case mb_begin_scan: //ахринеть, таймаут к началу арбитража ещё не кончился, а байт кто-то передал
			//подумаю завтра что делать в таких случаях
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
					"\n!!error end transmit mb_begin_scan!!\n");
			break;
		case mb_next_scan: //ахринеть, таймаут к началу арбитража команды следующего сканирования ещё не кончился, а байт кто-то передал
			//подумаю завтра что делать в таких случаях
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
					"\n!!error end transmit mb_next_scan!!\n");
			break;
		case mb_run_arbitrage://передавали Доминантное состояние (передаётся значением 0xFF)
		case mb_next_arbitrage:	//передавали Доминантное состояние (передаётся значением 0xFF)
			//надо перейти на приём
			fastmodbus_RecieveMode();
			break;
		default:
			critical_stop();
		}
	}
#ifdef NMBS_DEBUG
	else if (huart == &huart1) {
		debug_uart_run = false;
		flush_debug(true);
	}
#endif
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_StatusTypeDef res;
	if (huart == &modbusUart) {
		MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG, "%ld uart %02x buf_rec:%d ",
				HAL_GetTick(), nmbs.msg.buf[nmbs.msg.buf_rec],
				nmbs.msg.buf_rec);
		switch (fast_mb_mode) {
		case mb_none:	//продолжаем обычный приём
			strobe_toggle();
			if (msg_buf_inc(&nmbs)) {
				switch (check_fast_modbus(&nmbs, 5)) {	//проверяем 5 байт
				case fast_mb_none: //продолжаем приём данных как обычно
					//restart 3.5 timer
					//по нормальному надо выключать таймер, если через HAL, то он стопается и запрещаются прерывания таймера
					if (HAL_TIM_Base_Stop_IT(&TimerFastMB) != HAL_OK) {
						// Starting Error
						critical_stop();
					}
					// Generate an update event to reload the Prescaler
					// and the repetition counter (only for advanced timer) value immediately
					TimerFastMB.Instance->EGR = TIM_EGR_UG;
					break;
					//Начало сканирования
					//Мастер отправляет в шину команду «Начать сканирование», которая фактически звучит: «Есть кто?».
					//Приняв эту команду, все устройства-слейвы на шине считают себя неотсканированными.
				case fast_mb_begin_scan: //надо перенастроить таймер на величину ожидания арбитража
					i_am_not_scaned = true;
					fast_mb_mode = mb_begin_scan;
				case fast_mb_next_scan: //команда продолжить сканирование практически такая же как и начать скнирование
					//разница лишь в отсутсвии установки  только отличается i_am_not_scaned=true
					//stop timer 3.5 word
					if (HAL_TIM_Base_Stop_IT(&TimerFastMB) != HAL_OK) {
						// Starting Error
						critical_stop();
					}
					if (HAL_TIM_Base_DeInit(&TimerFastMB) != HAL_OK) {
						critical_stop();
					}
					MX_TIM6_Init(nmbs.msg.old_arbitrage ? 3 : 1);//инициализируем таймер на ожидание начала арбитража
					// TIM_EGR_UG есть внутри HAL_TIM_Base_Init, который вызывает TIM_Base_SetConfig
					// поэтому пока комментируем здесь эту операцию reload
					// Generate an update event to reload the Prescaler
					// and the repetition counter (only for advanced timer) value immediately
					//из-за бага в TIM_Base_SetConfig это закомментировано, поэтому открываем здесь
					TimerFastMB.Instance->EGR = TIM_EGR_UG;

					if (fast_mb_mode != mb_begin_scan) {
						MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
								"%ld start timer mb_next_scan\n",
								HAL_GetTick());
						fast_mb_mode = mb_next_scan;
					} else {
						MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
								"%ld start timer\n!!YES mb_begin_scan\n",
								HAL_GetTick());
					}
					arbitrage_window = 0;
					arbitrage_loss = false;
					make_arbitrage_data(&nmbs); //сгенерировать арбитражное 32 битное значение
					//включаем передатчик
					//SetRS485Transmit(); пока сделаем это только при передаче
					strobe_toggle();
					break;
				default:
					critical_stop();
				}
				//внутри предыдущего switch везде выполняется HAL_TIM_Base_Stop_IT а затем перезагружается TIM_EGR_UG
				// Check if the update flag is set after the Update Generation, if so clear the UIF flag
				// проверка и сброс должны быть подальше(пониже) относительно установки TIM_EGR_UG
				// because the timer runs a little slower than the CPU
				// https://community.st.com/t5/stm32-mcus-embedded-software/bug-in-tim-base-setconfig-fix-tim-base-setconfig-to-block-first/m-p/754265
				/*
				 if (HAL_IS_BIT_SET(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE)) {
				 // Clear the update flag
				 CLEAR_BIT(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE);
				 }
				 */
				//while (!HAL_IS_BIT_SET(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE));
				//CLEAR_BIT(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE);
				clear_tim_flag();
				if (HAL_TIM_Base_Start_IT(&TimerFastMB) != HAL_OK) {
					/* Starting Error */
					critical_stop();
				}
				//Receive next symbol
				if (HAL_UART_Receive_IT(&modbusUart,
						&nmbs.msg.buf[nmbs.msg.buf_rec], 1) != HAL_OK) {
					MP_FMB_DEBUG_PRINT(DEBUG_ERROR,
							"HAL_UART_Receive_IT error\n");
					critical_stop();
				}
			} else { //overflow input buffer
				MP_FMB_DEBUG_PRINT(DEBUG_INFO, "\n!!overflow input buffer!!\n");
				//stop timer 3.5 word
				if (HAL_TIM_Base_Stop_IT(&TimerFastMB) != HAL_OK) {
					// Starting Error
					critical_stop();
				}
				nano_RecieveMode();
			}
			break;
		case mb_begin_scan: //ахринеть, таймаут к началу арбитража ещё не кончился, а байт уже приняли
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\n!!mb_begin_scan error!!\n");
			//подумаю завтра что делать в таких случаях
			//break;
		case mb_next_scan://ахринеть, таймаут к началу арбитража продолжить сканирование ещё не кончился, а байт уже приняли
			//подумаю завтра что делать в таких случаях
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\n!!!mb_next_scan error!!\n");
			//пока заглатываем символ и бежим дальше
			//Receive next symbol
			if (modbusUart.gState == HAL_UART_STATE_READY) {
				res = HAL_UART_Receive_IT(&modbusUart,
						&nmbs.msg.buf[nmbs.msg.buf_rec], 1);
				if (res != HAL_OK) {
					MP_FMB_DEBUG_PRINT(DEBUG_ERROR,
							"HAL_UART_Receive_IT error\n");
					critical_stop();
				}
			} else {
				MP_FMB_DEBUG_PRINT(DEBUG_ERROR,
						"error state:%ld RxCpltCallback mb_next_scan HAL_UART_Receive_IT\n",
						modbusUart.gState);
			}
			break;
		case mb_run_arbitrage:
		case mb_next_arbitrage:
			if (arbitrage_window == 0) { //тоже странная ситуация.
				//таймаут к началу арбитража ещё не кончился, а байт уже приняли
				//подумаю завтра что делать в таких случаях
				//пока заглатываем символ и бежим дальше
				//Receive next symbol
				MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
						"\n!!!arbitrage_window not start!!\n");
				if (modbusUart.gState == HAL_UART_STATE_READY) {
					res = HAL_UART_Receive_IT(&modbusUart,
							&nmbs.msg.buf[nmbs.msg.buf_rec], 1);
					if (res != HAL_OK) {
						MP_FMB_DEBUG_PRINT(DEBUG_ERROR,
								"HAL_UART_Receive_IT error\n");
						critical_stop();
					}
				} else {
					MP_FMB_DEBUG_PRINT(DEBUG_ERROR,
							"error state:%ld RxCpltCallback mb_next_arbitrage HAL_UART_Receive_IT\n",
							modbusUart.gState);
				}
			} else {
				// arbitrage_window - 1 = это номер арбитражного окна, в котором приняли байт
				if (!arbitrage_loss) {
					if (arbitrage_word
							& (0x80000000 >> (arbitrage_window - 1))) { //1 - рециссивное состояние — это молчание в
						//течение арбитражного окна, если обнаружили передачу - проиграли
						MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,
								"w%02d %ld \n!loss!\n ", arbitrage_window,
								HAL_GetTick());
						arbitrage_loss = true;
					}
				} else {
					MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG, "w%02d %ld loss ",
							arbitrage_window, HAL_GetTick()); //мы уже проиграли арбитраж, поэтому тихо поём о поражении
				}
			}
			/* смысла нет арбитраж складывать в буфер
			 if (msg_buf_inc(&nmbs)) {
			 //Receive next symbol
			 if (HAL_UART_Receive_IT(&modbusUart,
			 &nmbs.msg.buf[nmbs.msg.buf_rec], 1) != HAL_OK) {
			 MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Receive_IT error\n");
			 critical_stop();
			 }
			 } else { //overflow input buffer

			 MP_FMB_DEBUG_PRINT(DEBUG_INFO,"\n!!fmb overflow!!\n");
			 nano_RecieveMode();
			 }
			 */
			//Receive next symbol
			if (HAL_UART_Receive_IT(&modbusUart, &nmbs.msg.buf[nmbs.msg.buf_rec], 1)
					!= HAL_OK) {
				MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "HAL_UART_Receive_IT error\n");
				critical_stop();
			}
			break;
		default:
			critical_stop();
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		uint32_t Size = msg_buf_get(&nmbs);
		switch (fast_mb_mode) {
		case mb_none:	//продолжаем обычный приём
			//нет смысла запускать процедуру обработки модбас при пустом входном буфере
			if (Size) { //number of received symbol
				MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\n%ld normal timer\n",
						HAL_GetTick());
				strobe_toggle();
				//stop timer 3.5 word
				if (HAL_TIM_Base_Stop_IT(&TimerFastMB) != HAL_OK) {
					// Starting Error
					critical_stop();
				}
				packet_sended = false; //надо знать была ли передача данных
				nmbs_error res_poll = nmbs_server_poll(&nmbs);
				if (NMBS_ERROR_NONE != res_poll) {
					MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
							"nmbs_server_poll error:%d size of receive:%ld\n",
							(int8_t) res_poll, Size);
					MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "%s\n",
							nmbs_strerror(res_poll));
					MP_DEBUG_DUMP(FM_LEVEL_DEBUG, (uint8_t* ) &nmbs,
							(uint16_t ) Size);
				}
				if (!packet_sended) {
					HAL_StatusTypeDef res;
					res = HAL_UART_AbortReceive(&modbusUart);
					if (res != HAL_OK) {
						MP_FMB_DEBUG_PRINT(DEBUG_ERROR,
								"HAL_UART_AbortReceive error %d\n", res);
						critical_stop();
					}
					nano_RecieveMode();
				}
			}
			break;
		case mb_begin_scan: //сработал таймер арбитража команды начала сканирования
		case mb_next_scan: //сработал таймер арбитража команды продолжения сканирования
			strobe_toggle();
			//переходим в режим арбитража: надо начать арбитраж и перенастроить таймер на арбитражное окно
			//stop timer arbitrage interval
			if (HAL_TIM_Base_Stop_IT(&TimerFastMB) != HAL_OK) {
				// Starting Error
				critical_stop();
			}
			if (HAL_TIM_Base_DeInit(&TimerFastMB) != HAL_OK) {
				critical_stop();
			}
			MX_TIM6_Init(nmbs.msg.old_arbitrage ? 4 : 2);//инициализируем таймер на арбитражное окно
			// TIM_EGR_UG есть внутри HAL_TIM_Base_Init, который вызывает TIM_Base_SetConfig
			// поэтому пока комментируем здесь эту операцию reload
			// Generate an update event to reload the Prescaler
			// and the repetition counter (only for advanced timer) value immediately
			//из-за бага в TIM_Base_SetConfig это закомментировано, поэтому открываем здесь
			TimerFastMB.Instance->EGR = TIM_EGR_UG;

			if (fast_mb_mode == mb_begin_scan) {

				MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,
						"%ld run timer win:%d\n", HAL_GetTick(),
						arbitrage_window);

				fast_mb_mode = mb_run_arbitrage; //в следующее прерывание сразу выйдем на второй арбитражный switch
			} else {
				MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,
						"%ld next timer win:%d\n", HAL_GetTick(),
						arbitrage_window);
				fast_mb_mode = mb_next_arbitrage; //в следующее прерывание сразу выйдем на второй арбитражный switch
			}

			// Check if the update flag is set after the Update Generation, if so clear the UIF flag
			// проверка и сброс должны быть подальше(пониже) относительно установки TIM_EGR_UG
			// because the timer runs a little slower than the CPU
			// https://community.st.com/t5/stm32-mcus-embedded-software/bug-in-tim-base-setconfig-fix-tim-base-setconfig-to-block-first/m-p/754265
			/*
			 if (HAL_IS_BIT_SET(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE)) {
			 // Clear the update flag
			 CLEAR_BIT(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE);
			 }
			 */

			//while (!HAL_IS_BIT_SET(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE));
			//CLEAR_BIT(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE);
			clear_tim_flag();

			if (HAL_TIM_Base_Start_IT(&TimerFastMB) != HAL_OK) {
				/* Starting Error */
				critical_stop();
			}

			//break; он здесь специально не нужен, чтобы обработать первое арбитражное окно
			//в следующем case сразу после окончания таймера начала арбитража
		case mb_run_arbitrage:
		case mb_next_arbitrage:
			strobe_toggle();
			if (arbitrage_window == 32) { //закончился арбитраж
				/* судя по анализу обмена никакого таймаута в этом случае нет, отправляем сразу по окончании арбитражного окна
				 убираем реинициализацию на 3.5, отправляем сразу после окончания арбитражного окна*/
				if (arbitrage_loss) {
					MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG, "\n%ld end arb win:%d\n",
							HAL_GetTick(), arbitrage_window);
				} else {
					MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
							"\n!!!WE WIN ARBITRAGE %ld end arb win:%d\n",
							HAL_GetTick(), arbitrage_window);
				}
				arbitrage_window++;
				break;
			} else if (arbitrage_window >= 33) { //таймер 3.5 секунды после того как закончился арбитраж сработал
				//stop timer 3.5 word
				if (HAL_TIM_Base_Stop_IT(&TimerFastMB) != HAL_OK) {
					// Starting Error
					critical_stop();
				}
				MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
						"\n %ld end arb tim win:%d\n", HAL_GetTick(),
						arbitrage_window);
				fast_mb_mode = mb_none; //после ответа (если он будет) продолжаем обычный приём
				if (!arbitrage_loss) { // если мы выиграли арбитраж, то надо ответить, сделаем процедуру для этого
					if (i_am_not_scaned) { //если мы ещё не отсканированы, то отвечаем такой командой
						//(1 байт) 0xFD широковещательный адрес
						//(1 байт) 0x46 команда работы с расширенными функциями
						//(1 байт) 0x03 субкоманда - признак ответа на сканирование
						//(4 байта) серийный номер устройства (big endian)
						//(1 байт) modbus адрес устройства
						//(2 байта) контрольная сумма
						i_am_not_scaned = false;	//устройство отсканировано
						answer_scan(&nmbs);
					} else { //если мы отсканированы и выиграли арбитраж (скорее всего отвечаем на команду продолжения сканирования)
						// все отсканированные устройства отправляют одно и то же сообщение «Конец сканирования»
						//(1 байт) 0xFD широковещательный адрес
						//(1 байт) 0x46 команда работы с расширенными функциями
						//(1 байт) 0x04 — субкоманда завершения сканирования;
						//(2 байта) xD3 0x93 — контрольная сумма.
						end_scan(&nmbs);
					}
				} else {
					if (HAL_UART_AbortReceive(&modbusUart) != HAL_OK) {
						critical_stop();
					}
					nano_RecieveMode();
				}
				break;
			}

			//Во время арбитража устройство-слейв передаёт по одному биту друг за другом арбитражное слово,
			//которое состоит из приоритета и уникального идентификатора.
			//
			//Приоритет сообщения — это 4 бита: 0 (0b0000) — наивысший, 15 (0b1111) — низший
			//
			//(28-битное число) младшие 28 бит уникального серийного номера
			//
			//Опрос событий — modbus-адрес (8-битное число). Устройства на шине уже настроены, коллизии адресов отсутствуют,
			//нет смысла тратить время на арбитраж по серийным номерам.
			//
			//В итоге арбитражное слово имеет длину 12 бит (4+8) при событиях или 32 бита (4+28) при сканировании

			//Ноль передаётся доминантным состоянием, а единица рецессивным — это значит, что арбитраж всегда выигрывают устройства,
			//у которых значение арбитражного слова меньше.
			//Так как 4 бита приоритета идут в начале, то более приоритетные сообщения выигрывают.
			//
			//Рецессивное состояние — это молчание в течение арбитражного окна
			//Доминантное состояние передаётся значением 0xFF

			//собственно принцип такой:
			//Если устройство должно передавать доминантное состояние, то по началу арбитражного окна оно отправляет в шину 0xFF.
			//Если на шине уже идет передача — устройство молчит, чтобы не передавать посылку, которая рассинхронизировалась.
			//В этом арбитражном окне такое устройство проиграть не может. Чужая передача обнаруживается с помощью флага BUS BUSY,
			//который есть в аппаратном блоке USART и выставляется, если на шине обнаружен чужой стартовый бит.

			//Если же устройство должно передать рецессивное состояние — оно молчит в течение всего арбитражного окна и слушает шину.
			//Если из шины за время арбитражного окна был принят байт — другое устройство передало доминантное состояние и арбитраж проигран.
			if (arbitrage_word & (0x80000000 >> arbitrage_window)) { //1 - рециссивное состояние — это молчание в
				MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG, "w%02d %ld silent ",
						arbitrage_window, HAL_GetTick());
				//течение арбитражного окна, если обнаружили передачу - проиграли
			} else { // Ноль - доминантным состоянием, надо передать 0xFF на шину, если ещё нет передачи
					 //даже если передача есть, то всё равно арбитраж продолжается - продолжаем "бороться"
					 //Bit 16 BUSY: Busy flag
					 //This bit is set and reset by hardware. It is active when a communication is ongoing on the
					 //RX line (successful start bit detected). It is reset at the end of the reception (successful or
					 //not).
					 //0: USART is idle (no reception)
					 //1: Reception on going

				//проверяем есть ли сейчас какая либо передача на линии
				if ((__HAL_UART_GET_FLAG(&modbusUart, UART_FLAG_BUSY) == SET)
						|| (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)
								== GPIO_PIN_RESET)) {
//				if ((__HAL_UART_GET_FLAG(&modbusUart, UART_FLAG_BUSY) == SET)) {
					MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,
							"w%02d %ld \n!!\n!!SET!!\n!!\n", arbitrage_window,
							HAL_GetTick());
				} else {
					static const uint8_t FF[1] = { 0xFF };
					//если мы ещё не проиграли арбитраж
					//то передаём доминантное состояние
					if (!arbitrage_loss) {
						//SetRS485Transmit(); //попробуем передатчик включить заранее
						strobe_toggle();
						write_serial(FF, 1, 0, &nmbs.platform.arg);
						MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG, "w%02d %ld FF ",
								arbitrage_window, HAL_GetTick());
					}
				}
			}
			arbitrage_window++; //следующее арбитражное окно
			break;
		default:
			critical_stop();
		}
	}
}
bool fast_mb_init() {
	//вычисление коэффициентов таймера в разных режимах.
	//Запускать всегда до вызова инициализации таймера
	compute_timer();
	fast_mb_mode = mb_none; //работаем как с обычным modbus
	uint32_t ver_hal = HAL_GetHalVersion();
	if (ver_hal != 0x1070800) {
		// Check if the update flag is set after the Update Generation, if so clear the UIF flag
		// проверка и сброс должны быть подальше(пониже) относительно установки TIM_EGR_UG
		// because the timer runs a little slower than the CPU
		// https://community.st.com/t5/stm32-mcus-embedded-software/bug-in-tim-base-setconfig-fix-tim-base-setconfig-to-block-first/m-p/754265
		//	 if (HAL_IS_BIT_SET(TimerFastMB.Instance->SR, TIM_FLAG_UPDATE))
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR, "CHECK BUG in HAL %08lx timer_set\n",
				ver_hal);
		return false;
	}
	return true;
}

