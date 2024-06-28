#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mz2048efh064.h>
#include <cp0defs.h> 
#include <stdio.h>
#include "RS485.h"

double powerVariable = 0.0;
double powerCheck = 0.0;
uint16_t adc = 0;
char adcStr[20];
bool bottomInsert = false;
bool topInsert = false;

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
    switch(message[1])
    {
        //Select Voltage on Shelf
        case 'V':
        {
            if(message[2] == 'O')
            {
                if(message[3] == '1')
                {
                    powerVariable = 3.3;
                }
                else if(message[3] == '2')
                {
                    powerVariable = 5.0;
                }
                else if (message[3] == '3')
                {
                    powerVariable = 12.0;
                }
                else
                {
                    powerVariable = 0.0;
                }
            }
        }
        
        //Select/Enable/Disable either the top or the bottom rack
        case 'I':
            if (message[2] == 'B')
            {
                if (message[3] == '1')
                {
                    LATFbits.LATF0 = 1;
                    LATFbits.LATF1 = 1;
                    LATFbits.LATF3 = 1;
                    LATFbits.LATF4 = 1;
                    LATFbits.LATF5 = 0;
                }
                else
                {
                    LATFbits.LATF0 = 0;
                    LATFbits.LATF1 = 0;
                    LATFbits.LATF3 = 1;
                    LATFbits.LATF4 = 0;
                    LATFbits.LATF5 = 0;
                }
            }
            if (message[2] == 'T')
            {
                if (message[3] == '1') 
                {
                    LATFbits.LATF0 = 1;
                    LATFbits.LATF1 = 1;
                    LATFbits.LATF3 = 0;
                    LATFbits.LATF4 = 0;
                    LATFbits.LATF5 = 1; 
                }
                else
                {
                    LATFbits.LATF0 = 0;
                    LATFbits.LATF1 = 0;
                    LATFbits.LATF3 = 0;
                    LATFbits.LATF4 = 0;
                    LATFbits.LATF5 = 0; 
                }
            }
            break;
            
        //Select position on addressed rack
        case 'P':
        {
            int position[32][5] = {
                {0,0,0,0,0},{0,0,0,0,1},{0,0,0,1,0},{0,0,0,1,1},{0,0,1,0,0},
                {0,0,1,0,1},{0,0,1,1,0},{0,0,1,1,1},{0,1,0,0,0},{0,1,0,0,1},
                {0,1,0,1,0},{0,1,0,1,1},{0,1,1,0,0},{0,1,1,0,1},{0,1,1,1,0},
                {0,1,1,1,1},{1,0,0,0,0},{1,0,0,0,1},{1,0,0,1,0},{1,0,0,1,1},
                {1,0,1,0,0},{1,0,1,0,1},{1,0,1,1,0},{1,0,1,1,1},{1,1,0,0,0},
                {1,1,0,0,1},{1,1,0,1,0},{1,1,0,1,1},{1,1,1,0,0},{1,1,1,0,1},
                {1,1,1,1,0},{1,1,1,1,1}
            }; 

            int num1 = message[2] - '0';
            int num2 = message[3] - '1';

            int num = num1 * 10 + num2;

            LATEbits.LATE0 = position[num][4];
            LATEbits.LATE1 = position[num][3];
            LATEbits.LATE2 = position[num][2];
            LATEbits.LATE3 = position[num][1];
            LATEbits.LATE4 = position[num][0];

            break;
        }
        
        //Enable various features on the backboard 
        case 'E':
            
            //Enable VTest
            if(message[2] == 'V') 
            {
                if(message[3] == '1') 
                {
                    LATDbits.LATD4 = 1;
                }
                else
                {
                    LATDbits.LATD4 = 0;
                }
            }
            
            //Enable RF
            if(message[2] == 'R') 
            {
                if(message[3] == '1')
                {
                    LATDbits.LATD5 = 1;
                }
                else
                {
                    LATDbits.LATD5 = 0;
                }
            }
            
            //Enable EFC
            if(message[2] == 'E') 
            {
                if(message[3] == '1') 
                {
                    LATDbits.LATD2 = 1;
                }
                else
                {
                    LATDbits.LATD2 = 0;
                }
            }
            
            //Enable Thermal addressing switch 
            if(message[2] == 'T') 
            {
                if(message[3] == '1') 
                {
                    LATDbits.LATD3 = 1;
                }
                else
                {
                    LATDbits.LATD3 = 0;
                }
            }   
            
        //Get a reading on one of the ADC channels
        case 'A':
            if(message[2] == 'V')
            {   
                //Voltage measurement
                if(message[3] == 'O') 
                {
                    adc = ADC_Read(4);
                    sprintf(adcStr, "%.2f", adc);
                    UART_TX_String(adcStr);
                }  
            }
            
            if(message[2] == 'T') 
            {
                //Top rack temperature sensor measurement
                if (message[3] == 'T') 
                {
                    adc = ADC_Read(3);
                    sprintf(adcStr, "%.2f", adc);
                    UART_TX_String(adcStr);
                }
            }
            
            if(message[2] == 'T') 
            {
                //Bottom rack temperature sensor measurement
                if(message[3] == 'B') 
                {
                    adc = ADC_Read(2);
                    sprintf(adcStr, "%.2f", adc);
                    UART_TX_String(adcStr);
                }
            }
            
            if(message[2] == 'I') 
            {
                //Current measurement of VOSC on bottom rack
                if(message[3] == 'B') 
                {
                    adc = ADC_Read(1);
                    sprintf(adcStr, "%.2f", adc);
                    UART_TX_String(adcStr);
                }
            }
            
            if(message[2] == 'I') 
            {
                //Current measurement of VOSC on top rack
                if(message[3] == 'T') 
                {
                    adc = ADC_Read(0);
                    sprintf(adcStr, "%.2f", adc);
                    UART_TX_String(adcStr);
                }
                break;  
            }
    }
}
//
//void __ISR(_EXTERNAL_1_VECTOR, IPL5AUTO) rackInsertTop(void)
//{
//    //Clear the interrupt flag
//    IFS0bits.INT1IF = 0;
//
//    //Toggle the state of bottomInsert
//    topInsert = !topInsert;
//    
//    //Toggle the edge detection mode
//    INTCONbits.INT1EP = !INTCONbits.INT1EP;
//    
//    //If the interrupt is the rack being inserted
//    if (topInsert)
//    {
//        if(bottomInsert)
//        {
//            //Enable the EN_SET1 and EN_SET2
//            LATBbits.LATB9 = 1;
//            LATBbits.LATB10 = 1;
//            
//            double checkVolt = ADC_Read(4);
//            if (checkVolt == powerVariable)
//            {
//                //Reset_TR to 0
//                LATBbits.LATB14 = 0;
//
//                //EN_VOSC TR to 1
//                LATBbits.LATB11 = 1;
//
//                //Gets the current reading from the bottom rack
//                double current = ADC_Read(0);
//
//                //Disable the EN_SET1 and EN_SET2
//                LATBbits.LATB9 = 0;
//                LATBbits.LATB10 = 0;
//            } 
//            else
//            {
//               //Reset_TR to 0
//                LATBbits.LATB14 = 1;
//
//                //EN_VOSC TR to 1
//                LATBbits.LATB11 = 0; 
//                
//                //Set the DISABLE and RESET buttons to 1
//                LATCbits.LATC15 = 1;
//                LATBbits.LATB15 = 1;
//            }
//        }
//        else
//        {
//            //Reset_TR to 0
//            LATBbits.LATB14 = 1;
//
//            //EN_VOSC TR to 1
//            LATBbits.LATB11 = 0; 
//        }
//    }
//    else
//    {        
//        //Reset_TR to 1
//        LATBbits.LATB14 = 1;
//
//        //EN_VOSC TR to 0
//        LATBbits.LATB11 = 0;
//    }
//}

void __ISR(_EXTERNAL_0_VECTOR, IPL5AUTO) rackInsertBottom(void)
{
    //Clear the interrupt flag
    IFS0bits.INT0IF = 0;
    
    //Toggle the edge detection mode
    INTCONbits.INT0EP = !INTCONbits.INT0EP;
    
    //Toggle the state of bottomInsert
    bottomInsert = !bottomInsert;
    
    //If the interrupt is the rack being inserted
    if (bottomInsert)
    {
        //Set the DISABLE and RESET buttons to 0
        LATCbits.LATC15 = 0;
        LATBbits.LATB15 = 0;
        
        //Reset_BR to 0
        LATBbits.LATB13 = 0;

        //Enable the EN_SET1 and EN_SET2
        LATBbits.LATB9 = 1;
        LATBbits.LATB10 = 1;

        if (powerVariable == 12.0)
        {
            LATBbits.LATB7 = 1;
            LATBbits.LATB8 = 1;
            powerCheck = 2.4; 
        }
        else if (powerVariable == 5.0)
        {
            LATBbits.LATB7 = 0;
            LATBbits.LATB8 = 1;
            powerCheck = 1.0; 
        }
        else if (powerVariable == 3.3)
        {
            LATBbits.LATB7 = 1;
            LATBbits.LATB8 = 0;
            powerCheck = 0.66; 
        }
        else
        {
            LATBbits.LATB7 = 0;
            LATBbits.LATB8 = 0;
        }
         double checkVolt = ADC_Read(4);
        
        if ((powerCheck - 0.1 <= checkVolt) && (checkVolt <= powerCheck + 0.1))
        {
            //EN_VOSC BR to 1
            LATBbits.LATB12 = 1;

            //Gets the current reading from the bottom rack
            double current = ADC_Read(1);

            //Disable the EN_SET1 and EN_SET2
            LATBbits.LATB9 = 0;
            LATBbits.LATB10 = 0;
        }   
        else
        {
            //Reset_BR to 1
            LATBbits.LATB13 = 1;

            //EN_VOSC BR to 0
            LATBbits.LATB12 = 0;
            
            //Set the DISABLE and RESET buttons to 1
            LATCbits.LATC15 = 1;
            LATBbits.LATB15 = 1;

            //Disable the EN_SET1 and EN_SET2
            LATBbits.LATB9 = 0;
            LATBbits.LATB10 = 0;
        }
    }
    else
    {      
        //Set the internal variable back to 0
        powerVariable = 0.0;
        
        //Set the DISABLE and RESET buttons to 0
        LATCbits.LATC15 = 0;
        LATBbits.LATB15 = 0;

        //Enable the EN_SET1 and EN_SET2
        LATBbits.LATB9 = 1;
        LATBbits.LATB10 = 1;
        
        //Set the power back to 0
        LATBbits.LATB7 = 0;
        LATBbits.LATB8 = 0;
        
        //Reset_BR to 1
        LATBbits.LATB13 = 1;

        //EN_VOSC BR to 0
        LATBbits.LATB12 = 0;
        
        //Set the DISABLE and RESET buttons to 1
        LATCbits.LATC15 = 1;
        LATBbits.LATB15 = 1;

        //Disable the EN_SET1 and EN_SET2
        LATBbits.LATB9 = 0;
        LATBbits.LATB10 = 0;
    }
}

double ADC_Read(uint8_t channel)
{
    if (channel >= 0 && channel <= 4)
    {
        uint16_t result[5]; 
        
        ADCCON3bits.ADINSEL = channel;  // Select the channel
        ADCCON3bits.SAMP = 1;           // Start sampling
        
        unsigned int i;
        for (i = 0; i < 20; i++) {}
        
        ADCCON3bits.RQCNVRT = 1;        // Start conversion
        ADCCON3bits.SAMP = 0;           // Release Sample
        while (ADCCON3bits.RQCNVRT);    // Wait for conversion to finish
        
        switch (channel) {
        case 0: return (ADCDATA0 * 3.3) / 4095.0; ;
        case 1: return (ADCDATA1 * 3.3) / 4095.0; ;
        case 2: return (ADCDATA2 * 3.3) / 4095.0; ;
        case 3: return (ADCDATA3 * 3.3) / 4095.0; ;
        case 4: return (ADCDATA4 * 3.3) / 4095.0; ;
        default: return 0;
        }
    }    
    else
    {
        return 0;     
    }
}

void handle_checksum_error(void) 
{
    return; // Exit the interrupt handler
}