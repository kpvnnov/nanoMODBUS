/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nanomodbus.h"
#include "fast_mb.h"
#include "fast_mb_port.h"

#ifdef COM_PORT_DEBUG
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <stdio.h>
//#define NMBS_DEBUG_DUMP(BUF,LEN) print_dump(BUF, LEN)
//#else
//#define NMBS_DEBUG_DUMP(...) (void) (0)

/*
 extern uint32_t FastModbus_Prescaler, Arbitrage_Period, Window_Period,
 Arbitrage_Periodx60, Window_Periodx60, Arbitrage_Periodx60,
 Window_Periodx60, Normal_Prescaler, Normal_Period;
 */

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

#define ToggleRS485Transmit() HAL_GPIO_TogglePin(USART2_RTS_GPIO_Port, USART2_RTS_Pin)

// The data model of this sever will support coils addresses 0 to 100 and registers addresses from 0 to 32
#define COILS_ADDR_MAX 100
#define REGS_ADDR_MAX 32

// Our RTU address
//#define RTU_SERVER_ADDRESS 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef modbusUart;
extern TIM_HandleTypeDef TimerFastMB;

uint32_t Speed = 96;
//uint32_t Speed = 192;
//uint32_t Speed = 384;
//uint32_t Speed = 576;
//uint32_t Speed = 1152;
uint16_t get_baudrate() {
	return Speed;
}
nmbs_t nmbs;
nmbs_arg_t nmbs_arg;
volatile bool packet_sended = false; //была ли в текущем цикле передача?
//volatile bool old_arbitrage;
//переменная отвечающая за включение отладки дергания ногой
volatile bool config_otladka_strobe = true;

volatile bool config_otladka_comport = true;

//смену скорости rs485 лучше сделать по окончании передачи пакета, когда поднимается этот флаг
volatile bool must_reload_rs485 = false;

// A single nmbs_bitfield variable can keep 2000 coils
nmbs_bitfield server_coils = { 0 };
uint16_t server_registers[REGS_ADDR_MAX + 1] = { 0 };
#ifdef COM_PORT_DEBUG

volatile bool debug_uart_run = false;
volatile uint16_t pos_print = 0;

uint8_t debug_buffer[4096];
volatile uint16_t pos_debug = 0;
//uint16_t print_debug = 0;

#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &TimerFastMB) {
		Timer_FastModbus(&nmbs);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &modbusUart) {
		UART_TxCplt(&nmbs);
	}
#ifdef COM_PORT_DEBUG
	else if (UART_Debug_Transmit(huart)) {
		return;
	}
#endif
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &modbusUart) {
		UART_RxCplt(&nmbs);
	}
}

#ifdef COM_PORT_DEBUG

bool UART_Debug_Transmit(UART_HandleTypeDef *huart) {
	if (huart == &UartDebug) {
		debug_uart_run = false;
		flush_debug();
		return true;
	}
	return false;
}

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
			HAL_UART_Transmit_IT(&UartDebug, &debug_buffer[pos_for_send],
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
			HAL_UART_Transmit_IT(&UartDebug, &debug_buffer[pos_for_send],
					len_for_send);

		}

	}

}
#endif

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

	packet_sended = true;
	HAL_StatusTypeDef res;
	//перенесли отмену приёма в таймер
	__HAL_ENTER_CRITICAL_SECTION();
	if (HAL_UART_STATE_BUSY_RX == modbusUart.RxState ) {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"!!!write_serial wrong HAL_UART_STATE_BUSY_RX\n");
		res = HAL_UART_AbortReceive(&modbusUart);
		if (res != HAL_OK) {
			MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"write_serial UART_AbortReceive error %d\n", res);
			critical_stop();
		}
	}
	if (modbusUart.gState == HAL_UART_STATE_READY) {
		SetRS485Transmit();
		res = HAL_UART_Transmit_IT(&modbusUart, buf, count);
		if (res != HAL_OK) {
			MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"HAL_UART_Transmit_IT error %d\n", res);
			critical_stop();
		}
	} else {
		MP_FMB_DEBUG_PRINT(DEBUG_ERROR,"error state:%ld write_serial HAL_UART_Transmit_IT \n", modbusUart.gState);
	}
	__HAL_EXIT_CRITICAL_SECTION();
	return count;
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

#ifdef COM_PORT_DEBUG

int _write(int file, char *data, int len) {
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
		errno = EBADF;
		return -1;
	}
//HAL_UART_Transmit(&huart1, (uint8_t*) data, (uint16_t) len, 0xFFFF);
//HAL_UART_Transmit_IT(&huart1, (uint8_t*) data, (uint16_t) len);
	int len_for_write = 0;
	int total_len = len;

	if ((pos_debug + len) >= sizeof(debug_buffer)) {
		__HAL_ENTER_CRITICAL_SECTION();
		int pos_for_write = pos_debug;
		int len_for_write = sizeof(debug_buffer) - pos_debug;
		pos_debug = 0;
		__HAL_EXIT_CRITICAL_SECTION();
		len -= len_for_write;
		strncpy(&debug_buffer[pos_for_write], data, len_for_write);

	}

	if (len > sizeof(debug_buffer)) {
		len = sizeof(debug_buffer);
	}

	__HAL_ENTER_CRITICAL_SECTION();
	int pos_for_write = pos_debug;
	len_for_write = len;
	pos_debug += len;
	if (pos_debug >= sizeof(debug_buffer)) {
		__HAL_EXIT_CRITICAL_SECTION();
		critical_stop();
		pos_debug = 0;
	}
	__HAL_EXIT_CRITICAL_SECTION();

	strncpy((char*) &debug_buffer[pos_for_write], data, len_for_write);

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
	MX_USART1_UART_Init();
	ModbusUart_Init();

	/* USER CODE BEGIN 2 */

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
	nmbs_fastmb_arg_create(&nmbs_arg);
	nmbs_arg.htim = &TimerFastMB; //указатель на таймер
	nmbs_arg.huart = &modbusUart;
	//nmbs_arg.Timer_FastModbus; //обработка прерываний таймера
	nmbs_arg.TIM_Start = Start_Timer; //запуск прерываний таймера
	nmbs_arg.TIM_Stop = Stop_Timer; //остановка прерываний таймера
	nmbs_arg.UART_Receive = Receive_Serial; //запуск приёма символов по rs485
	nmbs_arg.UART_AbortReceive = Abort_Serial; //остановка приема
	nmbs_arg.TIM_ReInit=TIM_ReStart; //реинициализация таймера на новый период

	nmbs_error err = nmbs_server_create(&nmbs, RTU_SERVER_ADDRESS,
			&platform_conf, &callbacks);
	if (err != NMBS_ERROR_NONE)
		onError(err);
	//запускать строго после nmbs_server_create
	nmbs_set_platform_arg(&nmbs, &nmbs_arg);
	if (!fast_mb_init(&nmbs)) {
		while (1) {
			RED_TOGGLE();
			HAL_Delay(250);
		}
	}
	nmbs.msg.fastmodbus_address = 4265607340; //(dec) или 0xFE4000AC 32 битный уникальный fastmodbus адрес устройства
//nmbs.msg.fastmodbus_address = 40;
//nmbs.msg.fastmodbus_address = 4294967040;
			//инициализация коэффициентов идёт в процедуре fast_mb_init
			//вычисление коэффициентов таймера в разных режимах.
			//Запускать всегда до вызова инициализации таймера
			//!!и после инициализации nmbs_arg
	if (MX_TIM_FastMB_Init(0, &nmbs)!= HAL_OK){
		while (1) {
			RED_TOGGLE();
			HAL_Delay(250);
		}
	}	


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
//run server in interrupt mode
	nano_RecieveMode(&nmbs);
	//((nmbs_arg_t*) nmbs.platform.arg)->nano_RecieveMode(&nmbs);
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//Here you can add the logic of the main program. At this point, modbus communication is in interrupt mode
		RED_TOGGLE();
#ifdef COM_PORT_DEBUG
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
