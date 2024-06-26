#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mz2048efh064.h>
#include <cp0defs.h> 
#include "RS485.h"

void __ISR(_UART1_RX_VECTOR, IPL7SRS) UART_RX(void)
{
    static uint8_t rx_array[RX_ARRAY_SIZE]; // Array to hold received data
    static uint8_t rx_array_ptr = 0; // Pointer to current position in array
    int i = 0;
    
    while(U1STAbits.URXDA)
    {
        rx_array[rx_array_ptr++] = U1RXREG;                                     // read byte
    }
    
    if (U1STAbits.OERR == 1)                                                    // check to see if an error was flagged 
    {  
        U1STAbits.OERR = 0;                                                     // clear error flag
    }
    
    else if(rx_array_ptr >= RX_ARRAY_SIZE)
    {
        rx_array_ptr = 0;
        for(i=0; i<RX_ARRAY_SIZE; i++)
        {
            rx_array[i] = 0;
        }
    }   
    
    else if (rx_array[rx_array_ptr-1] == '\n') 
    {
        IFS3bits.U1RXIF = 0; // Clear RX interrupt flag

        rx_array[rx_array_ptr++] = U1RXREG; // Read received data into array
        
        if (rx_array_ptr == MESSAGE_LENGTH+1) // Check if message is complete
        { 
            uint8_t address = rx_array[0]; // Get address from message
            uint8_t checksum = rx_array[CHECKSUM_INDEX]; // Get checksum from message

            uint8_t calculated_checksum = 0; // Variable for calculated checksum
            for (uint8_t i = 1; i < CHECKSUM_INDEX; i++) // Calculate checksum
            {
                calculated_checksum ^= rx_array[i]; // XOR each byte
            }     
            if (calculated_checksum == checksum) // Check if checksum is correct
            {
                if (address == MY_ADDRESS) // Check if address matches
                {
                    process_message(rx_array, MESSAGE_LENGTH); // Process message
                }
            } 
            else 
            {
                handle_checksum_error(); // Handle checksum error
                rx_array_ptr = 0; // Reset array pointer and exit interrupt
                return;
            }
            rx_array_ptr = 0; // Reset array pointer
        } 
    }
}
    

void UART_TX_Char(char c) {
    while (U1STAbits.UTXBF); // Wait if the transmit buffer is full
    U1TXREG = c; // Transmit the character
}

void UART_TX_String(const char *str) {
    while (*str != '\0') {
        UART_TX_Char(*str);
        str++;
    }
}

void process_message(uint8_t* message, uint8_t length) 
{
    switch(message[1]) // Switch on message type
    {   
        case 'W':
            // Check the third byte to determine the action for 'W' type message
            if(message[2] == '1') // Check if the command is '1'
            {
                LATDbits.LATD1 = 1; // Set LATD1 bit high
            }
            else
            {
                LATDbits.LATD1 = 0; // Set LATD1 bit low
            }
        break;

        case 'S':
        {
            // Define a position matrix with 8 sets of 3-bit positions
            int position[8][3] = {
                {0,0,0},{0,0,1},{0,1,0},{0,1,1},
                {1,0,0},{1,0,1},{1,1,0},{1,1,1}
            };

            // Convert the third byte of the message to an integer
            int num = message[2] - '1';

            // Set the LATD bits based on the position matrix
            LATDbits.LATD9 = position[num][2]; // Set LATD9
            LATDbits.LATD10 = position[num][1]; // Set LATD10
            LATDbits.LATD11 = position[num][0];  // Set LATD11
        }
        break;
    }
}

void handle_checksum_error(void) 
{
    return; // Exit the interrupt handler
}