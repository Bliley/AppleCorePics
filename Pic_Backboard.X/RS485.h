#ifndef INIT_H
#define INIT_H

#include <stdbool.h>

#define RX_ARRAY_SIZE 7
#define MESSAGE_LENGTH 6
#define CHECKSUM_INDEX 4 
#define MY_ADDRESS 0x09

void UART_TX_Char(char c);
void UART_TX_String(const char *str);
void process_message(uint8_t* message, uint8_t length);
double ADC_Read(uint8_t channel);
void handle_checksum_error(void);

#endif