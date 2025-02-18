/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 nanoModbus stm32 expamle irq
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "..\..\..\..\nanomodbus.h"
#ifdef NMBS_DEBUG
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <stdio.h>
#define NMBS_DEBUG_DUMP(BUF,LEN) print_dump(BUF, LEN)
#else
#define NMBS_DEBUG_DUMP(...) (void) (0)
#endif

//#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//invert the red LED
#define RED_TOGGLE()    HAL_GPIO_TogglePin(LED1_SYS_AL_GPIO_Port, LED1_SYS_AL_Pin)
#ifdef NMBS_DEBUG

inline void strobe_toggle() {
	if (get_debug_comport()) {
		HAL_GPIO_TogglePin(STROBE_GPIO_Port, STROBE_Pin);
		MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"%ld strobe ",HAL_GetTick());
	}
}
#else
#define strobe_toggle() (void) (0)
#endif

inline void clear_tim_flag() {
	volatile uint8_t counter = 100;
	while (!HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE) && counter--) {
		if (counter == 0) {
			MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"\n!!BUG clear_tim_flag!!\n ");
			critical_stop();
		}
	}
	if (counter != 100)
		MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"\n!!!clear_tim_flag %d ",counter);
	CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
}

#define SetRS485Receive() HAL_GPIO_WritePin(USART2_RTS_GPIO_Port, USART2_RTS_Pin, GPIO_PIN_RESET)
#define SetRS485Transmit() HAL_GPIO_WritePin(USART2_RTS_GPIO_Port, USART2_RTS_Pin, GPIO_PIN_SET)
#define ToggleRS485Transmit() HAL_GPIO_TogglePin(USART2_RTS_GPIO_Port, USART2_RTS_Pin)

// The data model of this sever will support coils addresses 0 to 100 and registers addresses from 0 to 32
#define COILS_ADDR_MAX 100
#define REGS_ADDR_MAX 32

// Our RTU address
#define RTU_SERVER_ADDRESS 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t Speed = 9600;
//uint32_t Speed = 19200;
//uint32_t Speed = 38400;
//uint32_t Speed = 57600;
//uint32_t Speed = 115200;
nmbs_t nmbs;
volatile bool packet_sended = false; //в текущем цикле была передача
//volatile bool old_arbitrage;
volatile bool config_otladka_comport = false;

uint32_t FastModbus_Prescaler, Arbitrage_Period, Window_Period,
		Arbitrage_Periodx60, Window_Periodx60, Arbitrage_Periodx60,
		Window_Periodx60, Normal_Prescaler, Normal_Period;

// A single nmbs_bitfield variable can keep 2000 coils
nmbs_bitfield server_coils = { 0 };
uint16_t server_registers[REGS_ADDR_MAX + 1] = { 0 };
#ifdef NMBS_DEBUG

volatile bool debug_uart_run = false;
volatile uint16_t pos_print = 0;

uint8_t debug_buffer[4096];
uint16_t pos_debug = 0;
//uint16_t print_debug = 0;

#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef NMBS_DEBUG

//void flush_debug(bool from_isr) {
void flush_debug() {
	if (debug_uart_run)
		return;
	if (pos_print != pos_debug) {
		//int len_for_send;
		volatile bool fast_quit = false;
		if (pos_print <= pos_debug) { //нормальный ход
			//if (!from_isr)
			//__disable_irq();
			__HAL_ENTER_CRITICAL_SECTION();
			int len_for_send = pos_debug - pos_print;
			int pos_for_send = pos_print;
			if (len_for_send == 0) {
				fast_quit = true;
			} else {
				pos_print += len_for_send;
			}

			//if (!from_isr)
			//	__enable_irq();
			__HAL_EXIT_CRITICAL_SECTION();
			if (fast_quit)
				return;

			if ((pos_for_send + len_for_send) > sizeof(debug_buffer)) {
				critical_stop();
			}
			debug_uart_run = true;
			HAL_UART_Transmit_IT(&huart1, &debug_buffer[pos_for_send],
					len_for_send);

		} else {    	//отправим до конца массива и сдвигаем указатель на ноль
			//if (!from_isr)
			//	__disable_irq();
			__HAL_ENTER_CRITICAL_SECTION();
			int len_for_send = sizeof(debug_buffer) - pos_print;
			int pos_for_send = pos_print;
			pos_print = 0;
			//if (!from_isr)
			//	__enable_irq();
			__HAL_EXIT_CRITICAL_SECTION();

			if ((pos_for_send + len_for_send) > sizeof(debug_buffer)) {
				critical_stop();
			}
			debug_uart_run = true;
			HAL_UART_Transmit_IT(&huart1, &debug_buffer[pos_for_send],
					len_for_send);

		}

	}

}
#endif

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

	Normal_Prescaler = SystemCoreClock / (1000000UL / 50UL) - 1;
	if (Speed > 19200) {
		Normal_Period = (1750 / 50) - 1;
	} else {
		Normal_Period = ((7UL * 1000000UL * 11UL / (50UL * 2UL)) / Speed); //-1 absent for rounding up
	}

	x = SystemCoreClock / (uint32_t) (Speed * 10);
	FastModbus_Prescaler = x - 1;
//вычисляем начало арбитража для новой команды 0x46
	if (Speed <= 38400) {
		k = 36;

	} else {
		k = 8UL * Speed / 10000UL + 1;
	}
	y = SystemCoreClock * k / (uint32_t) (Speed * x);
	Arbitrage_Period = y - 1;

	//вычисляем начало арбитража для старой команды 0x60
	k = 44;
	y = SystemCoreClock * k / (uint32_t) (Speed * x);
	Arbitrage_Periodx60 = y - 1;

	//длина арбитражного окна для новой команды 0x46
	k = 12UL + 5UL * Speed / 100000UL + 1;
	y = SystemCoreClock * k / (uint32_t) (Speed * x);
	Window_Period = y - 1;

	//длина арбитражного окна для старой команды 0x60
	k = 20;
	y = SystemCoreClock * k / (uint32_t) (Speed * x);
	Window_Periodx60 = y - 1;

}

// set counter of received symbols
void msg_buf_set(nmbs_t *nmbs, uint32_t length) {
	nmbs->msg.buf_rec = length;
}
// read counter received symbols
uint32_t msg_buf_get(nmbs_t *nmbs) {
	return nmbs->msg.buf_rec;
}
// incremet counter received symbols
bool msg_buf_inc(nmbs_t *nmbs) {
	if ((sizeof(nmbs->msg.buf) - nmbs->msg.buf_rec - 1) <= 0)
		return false;
	nmbs->msg.buf_rec++;
	return true;
}
typedef enum {
	fast_mb_none = 0x00,	//продолжаем обычный приём
	fast_mb_begin_scan = 0x01, //начать сканирование
	fast_mb_next_scan = 0x02, //продолжить сканирование
	fast_mb_answer_scan = 0x03,
	fast_mb_end_scan = 0x04,
	fast_mb_emulate = 0x08, //субкоманда эмуляции стандартных запросов
} fast_mb_command;

typedef enum {
	mb_none = 0x00,	//продолжаем обычный приём
	mb_begin_scan = 0x01, //перешли в режим арбитража после команды начала сканирования
	mb_next_scan = 0x02, //перешли в режим арбитража после команды продолжить скнирование
	mb_run_arbitrage = 0x03, //обрабатываем арбитражные окна после команды начала сканирования
	mb_next_arbitrage = 0x04, //обрабатываем арбитражные окна после команды продолжить сканирование

} modbus_mode;
modbus_mode fast_mb_mode; //что делаем при приемё команды fastmodbus
uint16_t arbitrage_window; //текущий номер арбитражного окна
uint32_t arbitrage_word; //32 битное арбитражное слово
//uint32_t fastmodbus_address = 4265607340; //(dec) или 0xFE4000AC 32 битный уникальный fastmodbus адрес устройства

bool i_am_not_scaned = false;
bool arbitrage_loss; //признак проигранного арбитража

void make_arbitrage_data(nmbs_t *nmbs) {
	if (i_am_not_scaned) {
		arbitrage_word = (0b0110 << 28)
				+ (nmbs->msg.fastmodbus_address & 0x0FFFFFFF);
	} else {
		//а у отсканированных — с низким (0b1111)
		arbitrage_word = (0b1111 << 28)
				+ (nmbs->msg.fastmodbus_address & 0x0FFFFFFF);
	}

}

fast_mb_command check_fast_modbus(nmbs_t *nmbs, uint8_t length) {
	if (length && nmbs->msg.buf_rec != length)
		return fast_mb_none;
	MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"got %d bytes\n",length);
	if (nmbs->msg.buf[0] != 0xFD)
		return fast_mb_none;
	if (nmbs->msg.buf[1] != 0x60 && nmbs->msg.buf[1] != 0x46)
		return fast_mb_none;
	MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"got broadcast\n");
	switch (nmbs->msg.buf[2]) {
	case 0x01: //Функция начала сканирования - 0x01
		//MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"fb func 1\n");
		if (nmbs->msg.buf[1] == 0x46 && nmbs->msg.buf[3] == 0x13
				&& nmbs->msg.buf[4] == 0x90) { //проверка crc
			nmbs->msg.old_arbitrage = false;
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"begin 0x46 fmb scan\n");
			return fast_mb_begin_scan;
		}
		if (ASK_OLD_FASTMODBUS && nmbs->msg.buf[1] == 0x60 && nmbs->msg.buf[3] == 0x09
				&& nmbs->msg.buf[4] == 0xF0) { //проверка crc
			nmbs->msg.old_arbitrage = true;
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"begin 0x60 fmb scan\n");
			return fast_mb_begin_scan;
		}
		break;
	case 0x02: // Функция продолжения сканирования - 0x02
		MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\nfb func 2\n");

		if (nmbs->msg.buf[1] == 0x46 && nmbs->msg.buf[3] == 0x53
				&& nmbs->msg.buf[4] == 0x91) { //проверка crc
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"next 0x46 fmb scan\n");
			return fast_mb_next_scan;
		}
		if (ASK_OLD_FASTMODBUS && nmbs->msg.buf[1] == 0x60 && nmbs->msg.buf[3] == 0x49
				&& nmbs->msg.buf[4] == 0xF1) { //проверка crc
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"next 0x46 fmb scan\n");
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
		MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"normal fb\n");
		return fast_mb_none;
	}
	return fast_mb_none;
}

// reset counter received symbols
void msg_rec_reset(nmbs_t *nmbs) {
	msg_buf_set(nmbs, 0);
}

int32_t read_from_buf(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms,
		void *arg) {

//in zero-copy mode buf_rec used for receive symbols in irq(interrupt) mode
//buf_idx increases with each read operation
//there is no need to create two receive buffers - for hardware reception and further reading
	return ((nmbs.msg.buf_rec - nmbs.msg.buf_idx - count) >= 0 ?
			count : (nmbs.msg.buf_rec - nmbs.msg.buf_idx));
}

int32_t write_serial(const uint8_t *buf, uint16_t count,
		int32_t byte_timeout_ms, void *arg) {
	SetRS485Transmit();
	packet_sended = true;
	HAL_StatusTypeDef res;
	res = HAL_UART_AbortReceive(&huart2);
	if (res != HAL_OK) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_AbortReceive error %d\n", res);
		critical_stop();
	}
	if (huart2.gState == HAL_UART_STATE_READY) {
		res = HAL_UART_Transmit_IT(&huart2, buf, count);
		if (res != HAL_OK) {
			MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Transmit_IT error %d\n", res);
			critical_stop();
		}
	} else {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"error state:%ld write_serial HAL_UART_Transmit_IT \n", huart2.gState);
	}
	return count;
}

void nano_RecieveMode(void) {

	/* Generate an update event to reload the Prescaler
	 and the repetition counter (only for advanced timer) value immediately */
	htim6.Instance->EGR = TIM_EGR_UG;

	SetRS485Receive();
	packet_sended = false;
	msg_rec_reset(&nmbs);
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n%ld nano_RecieveMode buf_rec:%d\n", HAL_GetTick()
			, nmbs.msg.buf_rec);
//Receive of data in IRQ Mode
	if (HAL_UART_Receive_IT(&huart2, nmbs.msg.buf, 1) != HAL_OK) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Receive_IT error\n");
		critical_stop();
	}
	/* Check if the update flag is set after the Update Generation, if so clear the UIF flag */
// проверка и сброс должны быть подальше(пониже) относительно установки TIM_EGR_UG
// because the timer runs a little slower than the CPU
// https://community.st.com/t5/stm32-mcus-embedded-software/bug-in-tim-base-setconfig-fix-tim-base-setconfig-to-block-first/m-p/754265
	/*

	 if (HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE)) {
	 // Clear the update flag
	 CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
	 }
	 */

	//while (!HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE))		;
	//CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
	clear_tim_flag();

	if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
		/* Starting Error */
		critical_stop();
	}
}
void fastmodbus_RecieveMode(void) {
	SetRS485Receive();
//Receive of data in IRQ Mode
	if (HAL_UART_Receive_IT(&huart2, nmbs.msg.buf, 1) != HAL_OK) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Receive_IT error\n");
		critical_stop();
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		switch (fast_mb_mode) {
		case mb_none:	//продолжаем обычный приём
			//after end of transmit go in receive mode
			nano_RecieveMode();
			break;
		case mb_begin_scan: //ахринеть, таймаут к началу арбитража ещё не кончился, а байт кто-то передал
			//подумаю завтра что делать в таких случаях
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n!!error end transmit mb_begin_scan!!\n");
			break;
		case mb_next_scan: //ахринеть, таймаут к началу арбитража команды следующего сканирования ещё не кончился, а байт кто-то передал
			//подумаю завтра что делать в таких случаях
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n!!error end transmit mb_next_scan!!\n");
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
	if (huart == &huart2) {
		MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"%ld uart %02x buf_rec:%d ", HAL_GetTick(),
				nmbs.msg.buf[nmbs.msg.buf_rec], nmbs.msg.buf_rec);
		switch (fast_mb_mode) {
		case mb_none:	//продолжаем обычный приём
			strobe_toggle();
			if (msg_buf_inc(&nmbs)) {
				switch (check_fast_modbus(&nmbs, 5)) {	//проверяем 5 байт
				case fast_mb_none: //продолжаем приём данных как обычно
					//restart 3.5 timer
					//по нормальному надо выключать таймер, если через HAL, то он стопается и запрещаются прерывания таймера
					if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK) {
						// Starting Error
						critical_stop();
					}
					// Generate an update event to reload the Prescaler
					// and the repetition counter (only for advanced timer) value immediately
					htim6.Instance->EGR = TIM_EGR_UG;
					/* выносим этот блок за пределы switch
					 // Check if the update flag is set after the Update Generation, if so clear the UIF flag
					 if (HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE)) {
					 // Clear the update flag
					 CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
					 MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"clear timer flag ");
					 }
					 выносим этот блок за пределы switch */
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
					if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK) {
						// Starting Error
						critical_stop();
					}
					if (HAL_TIM_Base_DeInit(&htim6) != HAL_OK) {
						critical_stop();
					}
					MX_TIM6_Init(nmbs.msg.old_arbitrage ? 3 : 1);//инициализируем таймер на ожидание начала арбитража
					// TIM_EGR_UG есть внутри HAL_TIM_Base_Init, который вызывает TIM_Base_SetConfig
					// поэтому пока комментируем здесь эту операцию reload
					// Generate an update event to reload the Prescaler
					// and the repetition counter (only for advanced timer) value immediately
					//из-за бага в TIM_Base_SetConfig это закомментировано, поэтому открываем здесь
					htim6.Instance->EGR = TIM_EGR_UG;

					/* выносим этот блок за пределы switch
					 // Check if the update flag is set after the Update Generation, if so clear the UIF flag
					 if (HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE)) {
					 // Clear the update flag
					 CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
					 }
					 if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
					 // Starting Error
					 critical_stop();
					 }
					 выносим этот блок за пределы switch */

					if (fast_mb_mode != mb_begin_scan) {
						MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"%ld start timer mb_next_scan\n", HAL_GetTick());
						fast_mb_mode = mb_next_scan;
					} else {
						MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"%ld start timer\n!!YES mb_begin_scan\n",
								HAL_GetTick());
					}
					arbitrage_window = 0;
					arbitrage_loss = false;
					make_arbitrage_data(&nmbs); //сгенерировать арбитражное 32 битное значение
					//включаем передатчик
					SetRS485Transmit();
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
				 if (HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE)) {
				 // Clear the update flag
				 CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
				 }
				 */
				//while (!HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE));
				//CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
				clear_tim_flag();

				if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
					/* Starting Error */
					critical_stop();
				}
				//Receive next symbol
				if (HAL_UART_Receive_IT(&huart2,
						&nmbs.msg.buf[nmbs.msg.buf_rec], 1) != HAL_OK) {
					MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Receive_IT error\n");
					critical_stop();
				}
			} else { //overflow input buffer
				MP_FMB_DEBUG_PRINT(DEBUG_INFO,"\n!!overflow input buffer!!\n");
				//stop timer 3.5 word
				if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK) {
					// Starting Error
					critical_stop();
				}
				nano_RecieveMode();
			}
			break;
		case mb_begin_scan: //ахринеть, таймаут к началу арбитража ещё не кончился, а байт уже приняли
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n!!mb_begin_scan error!!\n");
			//подумаю завтра что делать в таких случаях
			//break;
		case mb_next_scan://ахринеть, таймаут к началу арбитража продолжить сканирование ещё не кончился, а байт уже приняли
			//подумаю завтра что делать в таких случаях
			MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n!!!mb_next_scan error!!\n");
			//пока заглатываем символ и бежим дальше
			//Receive next symbol
			if (huart2.gState == HAL_UART_STATE_READY) {
				res = HAL_UART_Receive_IT(&huart2,
						&nmbs.msg.buf[nmbs.msg.buf_rec], 1);
				if (res != HAL_OK) {
					MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Receive_IT error\n");
					critical_stop();
				}
			} else {
				MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"error state:%ld RxCpltCallback mb_next_scan HAL_UART_Receive_IT\n", huart2.gState);
			}
			break;
		case mb_run_arbitrage:
		case mb_next_arbitrage:
			if (arbitrage_window == 0) { //тоже странная ситуация.
				//таймаут к началу арбитража ещё не кончился, а байт уже приняли
				//подумаю завтра что делать в таких случаях
				//пока заглатываем символ и бежим дальше
				//Receive next symbol
				MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n!!!arbitrage_window not start!!\n");
				if (huart2.gState == HAL_UART_STATE_READY) {
					res = HAL_UART_Receive_IT(&huart2,
							&nmbs.msg.buf[nmbs.msg.buf_rec], 1);
					if (res != HAL_OK) {
						MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Receive_IT error\n");
						critical_stop();
					}
				} else {
					MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"error state:%ld RxCpltCallback mb_next_arbitrage HAL_UART_Receive_IT\n", huart2.gState);
				}
			} else {
				// arbitrage_window - 1 = это номер арбитражного окна, в котором приняли байт
				if (!arbitrage_loss) {
					if (arbitrage_word
							& (0x8000000 >> (arbitrage_window - 1))) { //1 - рециссивное состояние — это молчание в
						//течение арбитражного окна, если обнаружили передачу - проиграли
						MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"w%02d %ld \n!loss!\n ", arbitrage_window,
								HAL_GetTick());
						arbitrage_loss = true;
					}
				} else {
					MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"w%02d %ld loss ", arbitrage_window, HAL_GetTick()); //мы уже проиграли арбитраж, поэтому тихо поём о поражении

				}
			}
			/* смысла нет арбитраж складывать в буфер
			 if (msg_buf_inc(&nmbs)) {
			 //Receive next symbol
			 if (HAL_UART_Receive_IT(&huart2,
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
			if (HAL_UART_Receive_IT(&huart2, &nmbs.msg.buf[nmbs.msg.buf_rec], 1)
					!= HAL_OK) {
				MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Receive_IT error\n");
				critical_stop();
			}
			break;
		default:
			critical_stop();
		}
	}
}

nmbs_error answer_scan(nmbs_t *nmbs);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		uint32_t Size = msg_buf_get(&nmbs);
		switch (fast_mb_mode) {
		case mb_none:	//продолжаем обычный приём
			//нет смысла запускать процедуру обработки модбас при пустом входном буфере
			if (Size) { //number of received symbol
				MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n%ld normal timer\n", HAL_GetTick());
				strobe_toggle();
				//stop timer 3.5 word
				if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK) {
					// Starting Error
					critical_stop();
				}
				packet_sended = false; //надо знать была ли передача данных
				nmbs_error res_poll = nmbs_server_poll(&nmbs);
				if (NMBS_ERROR_NONE != res_poll) {
					MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,
							"nmbs_server_poll error:%d size of receive:%ld\n",
							(int8_t ) res_poll, Size);
					MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"%s\n", nmbs_strerror(res_poll));
					MP_DEBUG_DUMP(FM_LEVEL_DEBUG, (uint8_t* ) &nmbs,
							(uint16_t ) Size);
				}
				if (!packet_sended) {
					HAL_StatusTypeDef res;
					res = HAL_UART_AbortReceive(&huart2);
					if (res != HAL_OK) {
						MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_AbortReceive error %d\n",
								res);
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
			if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK) {
				// Starting Error
				critical_stop();
			}
			if (HAL_TIM_Base_DeInit(&htim6) != HAL_OK) {
				critical_stop();
			}
			MX_TIM6_Init(nmbs.msg.old_arbitrage ? 4 : 2);//инициализируем таймер на арбитражное окно
			// TIM_EGR_UG есть внутри HAL_TIM_Base_Init, который вызывает TIM_Base_SetConfig
			// поэтому пока комментируем здесь эту операцию reload
			// Generate an update event to reload the Prescaler
			// and the repetition counter (only for advanced timer) value immediately
			//из-за бага в TIM_Base_SetConfig это закомментировано, поэтому открываем здесь
			htim6.Instance->EGR = TIM_EGR_UG;

			if (fast_mb_mode == mb_begin_scan) {

				MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"%ld run timer win:%d\n", HAL_GetTick(),
						arbitrage_window);

				fast_mb_mode = mb_run_arbitrage; //в следующее прерывание сразу выйдем на второй арбитражный switch
			} else {
				MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"%ld next timer win:%d\n", HAL_GetTick(),
						arbitrage_window);
				fast_mb_mode = mb_next_arbitrage; //в следующее прерывание сразу выйдем на второй арбитражный switch
			}

			// Check if the update flag is set after the Update Generation, if so clear the UIF flag
			// проверка и сброс должны быть подальше(пониже) относительно установки TIM_EGR_UG
			// because the timer runs a little slower than the CPU
			// https://community.st.com/t5/stm32-mcus-embedded-software/bug-in-tim-base-setconfig-fix-tim-base-setconfig-to-block-first/m-p/754265
			/*
			 if (HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE)) {
			 // Clear the update flag
			 CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
			 }
			 */

			//while (!HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE));
			//CLEAR_BIT(htim6.Instance->SR, TIM_FLAG_UPDATE);
			clear_tim_flag();

			if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
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
					MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n%ld end arb win:%d\n", HAL_GetTick(),
							arbitrage_window);
				} else {
					MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n!!!WE WIN ARBITRAGE %ld end arb win:%d\n",
							HAL_GetTick(), arbitrage_window);
				}
				arbitrage_window++;
				break;
			} else if (arbitrage_window >= 33) { //таймер 3.5 секунды после того как закончился арбитраж сработал
				//stop timer 3.5 word
				if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK) {
					// Starting Error
					critical_stop();
				}
				MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"\n %ld end arb tim win:%d\n", HAL_GetTick(),
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
					if (HAL_UART_AbortReceive(&huart2) != HAL_OK) {
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
			if (arbitrage_word & (0x8000000 >> arbitrage_window)) { //1 - рециссивное состояние — это молчание в
				MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"w%02d %ld silent ", arbitrage_window, HAL_GetTick());
				//течение арбитражного окна, если обнаружили передачу - проиграли
			} else { // Ноль - доминантным состоянием, надо передать 0xFF на шину, если ещё нет передачи
					 //даже если передача есть, то всё равно арбитраж продолжается - продолжаем "бороться"
					 //Bit 16 BUSY: Busy flag
					 //This bit is set and reset by hardware. It is active when a communication is ongoing on the
					 //RX line (successful start bit detected). It is reset at the end of the reception (successful or
					 //not).
					 //0: USART is idle (no reception)
					 //1: Reception on going

				if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_BUSY) == SET) { //проверяем есть ли сейчас какая либо передача на линии
					MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"w%02d %ld \n!!\n!!SET!!\n!!\n", arbitrage_window,
							HAL_GetTick());
				} else {
					static const uint8_t FF[1] = { 0xFF };
					//если мы ещё не проиграли арбитраж
					//то передаём доминантное состояние
					if (!arbitrage_loss) {
						write_serial(FF, 1, 0, &nmbs.platform.arg);
						MP_FMB_DEBUG_PRINT(FM_LEVEL_HIGH_DEBUG,"w%02d %ld FF ", arbitrage_window, HAL_GetTick());
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

nmbs_error handle_read_coils(uint16_t address, uint16_t quantity,
		nmbs_bitfield coils_out, uint8_t unit_id, void *arg) {
	if (address + quantity > COILS_ADDR_MAX + 1)
		return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

// Read our coils values into coils_out
	for (int i = 0; i < quantity; i++) {
		bool value = nmbs_bitfield_read(server_coils, address + i);
		nmbs_bitfield_write(coils_out, i, value);
	}
	return NMBS_ERROR_NONE;
}
char* get_string_module() {
	return "6DO8DI";
}

nmbs_error read_input_holding(bool is_holding, uint16_t address,
		uint16_t quantity, uint16_t *registers_out) {
	uint8_t shift_address;
	if (quantity >= 256)
		return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
	for (uint16_t i = 0; i < (quantity); i++) {
		uint16_t cur_addr = i + address;

		switch (cur_addr) {
		case 0:

			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
			break;
		case 17: //чтение DO в одном слове

			break;
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 24:
		case 25:
		case 26:
		case 27:
		case 28:
		case 29:
		case 30:
		case 31:

			break;
//чтение DI в одном слове @todo
		case 32:

			break;
		case 33:
		case 34:
		case 35:
		case 36:
		case 37:
		case 38:
		case 39:
		case 40:
		case 41:
		case 42:
		case 43:
		case 44:
			shift_address = 33;
			//чтение AO
			//надо выдать значение входов AO - хранятся в server_registers начиная с нулевой ячейки

			break;
		case 45:
		case 46:
		case 47:
		case 48:
		case 49:
		case 50:
		case 51:
		case 52:
		case 53:
		case 54:
		case 55:
		case 56:
		case 57:
		case 58:
		case 59:
		case 60:
		case 61:
		case 62:
		case 63:
		case 64:
		case 65:
		case 66:
		case 67:
		case 68:
		case 69:
		case 70:
		case 71:
		case 72:
		case 73:
		case 74:
		case 75:
		case 76:
			break;
		case 77:
		case 78:
		case 79:
		case 80:
		case 81:
		case 82:
		case 83:
		case 84:
		case 85:
		case 86:
		case 87:
		case 88:
		case 89:
		case 90:
		case 91:
		case 92:
		case 93:
		case 94:
		case 95:
		case 96:
		case 97:
		case 98:
		case 99:
		case 100:
		case 101:
		case 102:
		case 103:
		case 104:

			break;

		case 105: //Время работы с момента загрузки u32 секунды младшая часть
			//static uint16_t high_time_counter;
			break;
		case 106: //Время работы с момента загрузки u32 секунды
			//старшая часть числа запомненная при считывании младшей части
			break;

		case 110: //Скорость порта RS-485
			break;
		case 111: //бит чётности порта RS-485
			break;
		case 112: //Количество стоп-битов порта RS-485
			break;
		case 128:
		case 200:
		case 201:
		case 202:
		case 203:
		case 204:
		case 205:
		case 206:
		case 207:
		case 208:
		case 209:
		case 210:
		case 211:
		case 212:
		case 213:
		case 214:
		case 215:
		case 216:
		case 217:
		case 218:
		case 219:
			shift_address = 200;
			//Модель устройства
			char *model = get_string_module();
			uint8_t length_name = strlen(model);
			//uint8_t shift_word=(cur_addr - shift_address) * 2;
			uint8_t shift_word = (cur_addr - shift_address);
			if (shift_word < length_name) {
				//registers_out[i]=model[shift_word]| (model[shift_word+1]<<8);
				registers_out[i] = model[shift_word];
			} else {
				registers_out[i] = 0;
			}
			break;
		case 270: //серийный номер

			registers_out[i] = nmbs.msg.fastmodbus_address & 0xFFFF;
			break;
		case 271: //серийный номер

			registers_out[i] = nmbs.msg.fastmodbus_address >> 16;
			break;
		case 320: //Версия прошивки в числовом формате MAJOR
			registers_out[i] = MAJOR_VER;
			break;
		case 321: //Версия прошивки в числовом формате MINOR
			registers_out[i] = MINOR_VER;
			break;
		case 322: //Версия прошивки в числовом формате PATCH
			registers_out[i] = PATCH_VER;
			break;
		case 323: //Версия прошивки в числовом формате SUFFIX, значение знаковое.
			registers_out[i] = SUFFIX_VER;
			break;
		case 324: //Версия прошивки в u32 VERSION = (MAJOR << 24) + (MINOR << 16) + (PATCH << 8) + SUFFIX;
			uint8_t suffix;
			if (SUFFIX_VER >= 0) {
				suffix = SUFFIX_VER + 128;
			} else {
				suffix = -1 - SUFFIX_VER;
			}
			registers_out[i] = (PATCH_VER << 8) + suffix;
			break;
		case 325: //Версия прошивки в u32
			registers_out[i] = (MAJOR_VER << 8) + MINOR_VER;
			break;
		case 330: //версия загрузчика

			break;
		case 331: //версия загрузчика

			break;

		default:
			if (is_holding) {

			}
			//для сквозной адресации ошибку не выдаём, а возвращаем 0
			//Для всех модулей ОДИНАКОВОЕ адресное пространство
			registers_out[i] = 0;
			//return NMBS_ERROR_NONE;
			//return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
		}
	}

	return NMBS_ERROR_NONE;

}

//0X03 read_holding_registers
nmbs_error handler_read_holding_registers(uint16_t address, uint16_t quantity,
		uint16_t *registers_out, uint8_t unit_id, void *arg) {
	return read_input_holding(true, address, quantity, registers_out);
}

void onError(nmbs_error err) {
	printf("error: %d\n", err);
	exit(0);
}

#ifdef NMBS_DEBUG

int _write(int file, char *data, int len) {
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
		errno = EBADF;
		return -1;
	}
//HAL_UART_Transmit(&huart1, (uint8_t*) data, (uint16_t) len, 0xFFFF);
//HAL_UART_Transmit_IT(&huart1, (uint8_t*) data, (uint16_t) len);
	int my_len = 0;
	int total_len = len;
	if ((pos_debug + len) >= sizeof(debug_buffer)) {
		//my_len = (pos_debug + len) - sizeof(debug_buffer);
		my_len = sizeof(debug_buffer) - pos_debug;
		strncpy(&debug_buffer[pos_debug], data, my_len);
		pos_debug = 0;
		len -= my_len;
	}
	if (len > sizeof(debug_buffer)) {
		len = sizeof(debug_buffer);
	}
	strncpy((char*) &debug_buffer[pos_debug], data, len);
	pos_debug += len;
	if (pos_debug >= sizeof(debug_buffer)) {

		critical_stop();
		pos_debug = 0;
	}
	if (!debug_uart_run) {
		flush_debug();
	}

	return total_len;
}

void print_dump(uint8_t *buf, uint16_t len) {
	uint16_t offset = 0;
	while (len) {
		printf("%04x ", offset);
		for (uint8_t x = 0; x < (len > 16 ? 16 : len); x++) {
			printf("%02x ", *buf++);

		}
		len -= len > 16 ? 16 : len;
		offset += 16;
		printf("\n");
	}
}

#endif

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
//вычисление коэффициентов таймера в разных режимах.
//Запускать всегда до вызова инициализации таймера
	compute_timer();
	MX_TIM6_Init(0);
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	fast_mb_mode = mb_none; //работаем как с обычным modbus
	uint32_t ver_hal = HAL_GetHalVersion();
	if (ver_hal != 0x1070800) {
		// Check if the update flag is set after the Update Generation, if so clear the UIF flag
		// проверка и сброс должны быть подальше(пониже) относительно установки TIM_EGR_UG
		// because the timer runs a little slower than the CPU
		// https://community.st.com/t5/stm32-mcus-embedded-software/bug-in-tim-base-setconfig-fix-tim-base-setconfig-to-block-first/m-p/754265
		//	 if (HAL_IS_BIT_SET(htim6.Instance->SR, TIM_FLAG_UPDATE))
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"CHECK BUG in HAL %08lx timer_set\n",ver_hal);
		while (1) {
			RED_TOGGLE();
			HAL_Delay(250);
		}
	}

//MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"Begin\n");
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"Speed:%ld Address:%d\n", Speed, RTU_SERVER_ADDRESS);
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"FastModbus_Prescaler:%ld\n", FastModbus_Prescaler);
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"    Arbitrage_Period:%5ld     Window_Period:%5ld\n",
			Arbitrage_Period, Window_Period);
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"Old Arbitrage_Period:%5ld old Window_Period:%5ld\n",
			Arbitrage_Periodx60, Window_Periodx60);
	MP_FMB_DEBUG_PRINT(FM_LEVEL_DEBUG,"Normal_Prescaler:%ld Normal_Period:%ld\n", Normal_Prescaler,
			Normal_Period);
	nmbs_platform_conf platform_conf;
	nmbs_platform_conf_create(&platform_conf);
	platform_conf.transport = NMBS_TRANSPORT_RTU;
	platform_conf.read = read_from_buf;
	platform_conf.write = write_serial;

	nmbs_callbacks callbacks;
	nmbs_callbacks_create(&callbacks);
	callbacks.read_coils = handle_read_coils;
//0x03
	callbacks.read_holding_registers = handler_read_holding_registers;

	nmbs_error err = nmbs_server_create(&nmbs, RTU_SERVER_ADDRESS,
			&platform_conf, &callbacks);
	nmbs.msg.fastmodbus_address = 4265607340; //(dec) или 0xFE4000AC 32 битный уникальный fastmodbus адрес устройства
//nmbs.msg.fastmodbus_address = 40;
//nmbs.msg.fastmodbus_address = 4294967040;
	if (err != NMBS_ERROR_NONE)
		onError(err);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
//run server in interrupt mode
	nano_RecieveMode();
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//Here you can add the logic of the main program. At this point, modbus communication is in interrupt mode
		RED_TOGGLE();
#ifdef NMBS_DEBUG
		//flush_debug(false);
#endif
		HAL_Delay(1000);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		critical_stop();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		critical_stop();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		critical_stop();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"Wrong parameters value: file %s on line %ld\r\n", file, line);
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
