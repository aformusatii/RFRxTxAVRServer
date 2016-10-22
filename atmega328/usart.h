
#ifndef USART_H_
#define USART_H_

/********************************************************************************
Includes
********************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include "../common/util.h"

/********************************************************************************
Macros and Defines
********************************************************************************/
// http://wormfood.net/avrbaudcalc.php?bitrate=300%2C600%2C1200%2C2400%2C4800%2C9600%2C14.4k%2C19.2k%2C28.8k%2C38.4k%2C57.6k%2C76.8k%2C115.2k%2C230.4k%2C250k%2C.5m%2C1m&clock=8&databits=8
#define BAUD 750000
#define MYUBRR F_CPU/16/BAUD-1

#define CONSOLE_PREFIX "\natmega328>"

#define MAX_CMD_ARGS  10

/********************************************************************************
Function Prototypes
********************************************************************************/
void usart_init();
char usart_getchar( void );
void usart_putchar( char data );
void usart_pstr (char *s);
unsigned char usart_kbhit(void);
int usart_putchar_printf(char var, FILE *stream);

void handle_usart_interrupt();
void usart_check_loop();
void handle_usart_cmd(char *cmd, char* args[], uint8_t arg_count);

#endif /* USART_H_ */
