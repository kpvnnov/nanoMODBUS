/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define NMBS_DEBUG 1
//выводим дополнительно изменения уровня на внешние ноги
//в рабочей версии ОТКЛЮЧАТЬ!!!!
#define NMBS_DEBUG_PIN (get_debug_comport() && NMBS_DEBUG)

//задаём уровень отладки fast modbus
#define FM_LEVEL_HIGH_DEBUG DEBUG_HIGH_TRACE
//#define FM_LEVEL_DEBUG DEBUG_TRACE
#define FM_LEVEL_DEBUG DEBUG_TRACE

#define DEBUG_ERROR 0
#define DEBUG_WARN  1
#define DEBUG_INFO  2
#define DEBUG_TRACE  3
#define DEBUG_HIGH_TRACE  4


#define get_verbose_debug(debug_level) (debug_level<=DEBUG_HIGH_TRACE)
//#define get_verbose_debug(debug_level) (debug_level<=config.otladka.verbose)

//включение отладки программным способом
//#define get_debug_comport() (config.otladka.comport)
#define get_debug_comport() (1)


#define BOOLEAN2INT (0x0001) //значение для integer значений, которые должны быть единицей
//#define BOOLEAN2INT (0xFFFF)

//версия прошивки @todo в будущем брать из автосборки
#define MAJOR_VER 1
#define MINOR_VER 0
#define PATCH_VER 11
//значение знаковое
#define SUFFIX_VER 0


#define __HAL_ENTER_CRITICAL_SECTION() \
    uint32_t PriMsk; \
    PriMsk = __get_PRIMASK(); \
    __set_PRIMASK(1);

#define __HAL_EXIT_CRITICAL_SECTION() \
    __set_PRIMASK(PriMsk);


// для хранения данных у нас есть server_registers
//с нулевой по 15 ячейку храним AO
#define REGS_ADDR_BOUND 16
//начиная с ячейки REGS_ADDR_BOUND и до ячейки REGS_HOLD_BOUND хранятся данные дискретных входов
#define REGS_HOLD_BOUND (REGS_ADDR_BOUND+16)
//в AI модуле необходимо будет ещё хранить данные с различных АЦП, поэтому это место пока в резерве
#define REGS_AI_BOUND (REGS_HOLD_BOUND+16)


#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_CLK_DISABLE()              __HAL_RCC_USART1_CLK_DISABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOB_CLK_DISABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOB
#define USARTx_TX_AF                     GPIO_AF0_USART1
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOB
#define USARTx_RX_AF                     GPIO_AF0_USART1
/* End definition for USARTx clock resources */

#ifdef NMBS_DEBUG
#include <stdio.h>
#define NMBS_DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define NMBS_DEBUG_PRINT(...) (void) (0)
#endif



#ifdef NMBS_DEBUG
#include <stdio.h>
//#define MP_DEBUG_PRINT(VERBOSE_LEVEL,FMT...) printf(__VA_ARGS__)
//#define MP_DEBUG_PRINT(VERBOSE_LEVEL,FMT...) if (get_verbose_debug(VERBOSE_LEVEL)) printf(__VA_ARGS__)
#define MP_DEBUG_PRINT(VERBOSE_LEVEL,FMT...) if (get_verbose_debug(VERBOSE_LEVEL) && get_debug_comport()) printf(FMT)
#define MP_DEBUG_DUMP(VERBOSE_LEVEL,BUF, LEN) if (get_verbose_debug(VERBOSE_LEVEL) && get_debug_comport()) print_dump(BUF, LEN)

#else
#define MP_DEBUG_PRINT(...) (void) (0)
#define MP_DEBUG_DUMP(VERBOSE_LEVEL,BUF, LEN) (void) (0)
#endif

#ifdef NMBS_DEBUG

#define FASTMODBUS_DEBUG
#ifdef FASTMODBUS_DEBUG
#define MP_FMB_DEBUG_PRINT MP_DEBUG_PRINT
#else
#define MP_FMB_DEBUG_PRINT(...) (void) (0)
#endif

#endif

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_RTS_Pin GPIO_PIN_1
#define USART2_RTS_GPIO_Port GPIOA
#define LED1_SYS_AL_Pin GPIO_PIN_11
#define LED1_SYS_AL_GPIO_Port GPIOA

#define STROBE_Pin GPIO_PIN_5
#define STROBE_GPIO_Port GPIOB


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
