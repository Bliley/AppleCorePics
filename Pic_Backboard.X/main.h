#ifndef MAIN_H
#define MAIN_H

void Sys_Init(void);
void init_uart(void);
void init_adc(void);
void init_interrupt(void);

#define UART_RX_INT_PRIORITY 1

#endif