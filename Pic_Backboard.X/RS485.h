#ifndef INIT_H
#define INIT_H

#include <stdint.h>
#include <stdbool.h>

#define RX_ARRAY_SIZE 7
#define MESSAGE_LENGTH 6
#define CHECKSUM_INDEX 4 
#define MY_ADDRESS 0x09

extern bool messageAvailable;
extern uint8_t messageBuf[MESSAGE_LENGTH];

void UART_TX_Char(char c);
void UART_TX_String(const char *str);
void process_message(uint8_t* message, uint8_t length);
double ADC_Read(uint8_t channel);
void handle_checksum_error(void);
void voltage_call(uint8_t* message);
void power_call(uint8_t* message);
void reset_call(uint8_t* message);
void bottomTop_call(uint8_t* message);
void position_call(uint8_t* message);
void enable_call(uint8_t* message);
void adc_call(uint8_t* message);
                
#endif