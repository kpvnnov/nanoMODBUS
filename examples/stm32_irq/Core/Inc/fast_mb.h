/**
 ******************************************************************************
 * @file           : fast_mb.h
 * @brief          : Header for fast_mb.c file.
 *                   This file contains the common defines of the fastmodbus port.
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
#ifndef __FASTMB_H__
#define __FASTMB_H__

#ifdef __cplusplus
extern "C" {
#endif

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

bool fast_mb_init();
void nano_RecieveMode(void);
void fastmodbus_RecieveMode(void);

//подсчёт всех интервалов для таймера, чтобы не терять время на расчёты коэффициентов в работе прерываний
void compute_timer();

#ifdef __cplusplus
}
#endif

#endif /* __FASTMB_H__ */

