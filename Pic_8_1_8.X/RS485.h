#ifndef INIT_H
#define INIT_H

#define RX_ARRAY_SIZE 6
#define MESSAGE_LENGTH 5
#define CHECKSUM_INDEX 3 
#define MY_ADDRESS 0x01

void UART_TX_Char(char c);
void UART_TX_String(const char *str);
void process_message(uint8_t* message, uint8_t length);
void handle_checksum_error(void);

#endif