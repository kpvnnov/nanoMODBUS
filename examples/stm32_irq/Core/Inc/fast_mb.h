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

uint16_t get_baudrate();
void nano_RecieveMode(void);
void fastmodbus_RecieveMode(void);

//подсчёт всех интервалов для таймера, чтобы не терять время на расчёты коэффициентов в работе прерываний
void compute_timer();

