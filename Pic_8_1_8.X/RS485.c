#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mz2048efh064.h>
#include <cp0defs.h> 
#include "main.h"
#include "RS485.h"

int i;
volatile bool messageAvailable = false;
uint8_t messageBuf[MESSAGE_LENGTH];

void __ISR(_UART1_RX_VECTOR, IPL1SRS) UART_RX(void)
{
    static uint8_t rx_array[RX_ARRAY_SIZE];                                     // Array to hold received data
    static uint8_t rx_array_ptr = 0;                                            // Pointer to current position in array
    
    while (U1STAbits.URXDA)
    {
        if (rx_array_ptr < RX_ARRAY_SIZE)
        {
            rx_array[rx_array_ptr++] = U1RXREG;                                 // Read byte
        }
        else
        {
            rx_array_ptr = 0;
            for (i = 0; i < RX_ARRAY_SIZE; i++)
            {
                rx_array[i] = 0;                                                // Clear array
            }
            break;
        }
    }

    if (rx_array[0] == MY_ADDRESS)
    {
        if (U1STAbits.OERR == 1)                                                // Check for overrun error
        { 
            U1STAbits.OERR = 0;                                                 // Clear error flag
        }
        else if (rx_array_ptr > 0 && rx_array[rx_array_ptr - 1] == '\n')        // Check for end of message
        { 
            IFS3bits.U1RXIF = 0;                                                // Clear RX interrupt flag

            if (rx_array_ptr == MESSAGE_LENGTH)                                 // Check if message is complete
            {
                uint8_t address = rx_array[0];                                  // Get address from message
                uint8_t checksum = rx_array[CHECKSUM_INDEX];                    // Get checksum from message

                uint8_t calculated_checksum = 0;                                // Variable for calculated checksum
                for (i = 0; i < CHECKSUM_INDEX; i++)
                {
                    calculated_checksum ^= rx_array[i];                         // XOR each byte
                }

                if (calculated_checksum == checksum)                            // Check if checksum is correct
                {
                    for (i = 0; i < MESSAGE_LENGTH; i++)
                    {
                        messageBuf[i] = rx_array[i];                            // Copy message to buffer
                    }
                    messageAvailable = true;                                    // Set the flag indicating a message is ready
                }
                else
                {
                    handle_checksum_error();                                    // Handle checksum error
                }

                rx_array_ptr = 0;
                for (i = 0; i < RX_ARRAY_SIZE; i++)
                {
                    rx_array[i] = 0;                                            // Clear array
                }

                return;                                                         // Exit the ISR
            }
        }
    }

    if (rx_array_ptr >= RX_ARRAY_SIZE || rx_array[0] != MY_ADDRESS || U1STAbits.OERR == 1)
    {
        rx_array_ptr = 0;
        for (i = 0; i < RX_ARRAY_SIZE; i++)
        {
            rx_array[i] = 0;                                                    // Clear array
        }
    }

    IFS3bits.U1RXIF = 0;                                                        // Clear RX interrupt flag
}
    

void UART_TX_Char(char c) {
    while (U1STAbits.UTXBF);                                                    // Wait if the transmit buffer is full
    U1TXREG = c;                                                                // Transmit the character
}

void UART_TX_String(const char *str) {
    while (*str != '\0') {
        UART_TX_Char(*str);
        str++;
    }
}

void process_message(uint8_t* message, uint8_t length) 
{
    switch(message[1])                                                          // Switch on message type
    {   
        case 'W':
                waveform_call(message);
                LATGbits.LATG6 = 1;
                for(i = 0; i < 1000000; i++){}                                    
                UART_TX_Char('P');
                for(i = 0; i < 1000000; i++){}                                    
                LATGbits.LATG6 = 0;
            break;
        case 'S':
                shelf_call(message);
                LATGbits.LATG6 = 1;
                for(i = 0; i < 1000000; i++){}                                    
                UART_TX_Char('P');
                for(i = 0; i < 1000000; i++){}                                    
                LATGbits.LATG6 = 0;
            break; 
        case 'T':
                test_call(message);
                LATGbits.LATG6 = 1;
                for(i = 0; i < 1000000; i++){}                                    
                UART_TX_Char('P');
                for(i = 0; i < 1000000; i++){}                                    
                LATGbits.LATG6 = 0;
            break;
        default:
                LATGbits.LATG6 = 1;
                for(i = 0; i < 1000000; i++){}                                    
                UART_TX_Char('F');
                for(i = 0; i < 1000000; i++){}                                    
                LATGbits.LATG6 = 0;
            break;
    }
}

void handle_checksum_error(void) 
{
    return;                                                                     // Exit the interrupt handler
}

void shelf_call(uint8_t* message)
{
    if (message[2] >= '0' && message[2] <= '9')
    {
        int position[8][3] = {                                                  // Define a position matrix with 8 sets of 3-bit positions
             {0,0,0},{0,0,1},{0,1,0},{0,1,1},
             {1,0,0},{1,0,1},{1,1,0},{1,1,1}
         };

         int num = message[2] - '1';                                            // Convert the third byte of the message to an integer

         LATDbits.LATD9 = position[num][2];                                     // Set LATD9 based on the position matrix
         LATDbits.LATD10 = position[num][1];                                    // Set LATD10 based on the position matrix
         LATDbits.LATD11 = position[num][0];                                    // Set LATD11 based on the position matrix
    }
}

void test_call(uint8_t* message)
{
    if (message[2] >= '0' && message[2] <= '9')
    {
        int position[8][3] = {                                                  // Define a position matrix with 8 sets of 3-bit positions
             {0,0,0},{0,0,1},{0,1,0},{0,1,1},
             {1,0,0},{1,0,1},{1,1,0},{1,1,1}
         };

         int num = message[2] - '1';                                            // Convert the third byte of the message to an integer

         LATDbits.LATD2 = position[num][2];                                     // Set LATD2 based on the position matrix
         LATDbits.LATD3 = position[num][1];                                     // Set LATD3 based on the position matrix
         LATDbits.LATD4 = position[num][0];                                     // Set LATD4 based on the position matrix
    }
}

void waveform_call(uint8_t* message)
{
    if(message[2] == '1')                                                       // Check if the command is '1'
    {
        LATDbits.LATD1 = 1;                                                     // Set LATD1 bit high
    }
    else
    {
        LATDbits.LATD1 = 0;                                                     // Set LATD1 bit low
    }
}