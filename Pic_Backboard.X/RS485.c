#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mz2048efh064.h>
#include <cp0defs.h>
#include <stdio.h>
#include "RS485.h"

int powerVariable = 0;
double powerCheck = 0.0;
uint16_t adc = 0;
char adcStr[20];
int i;
double checkVolt = 0;
char checkVoltStr[20];

void __ISR(_UART1_RX_VECTOR, IPL1SRS) UART_RX(void)
{
    static uint8_t rx_array[RX_ARRAY_SIZE];                                     // Array to hold received data
    static uint8_t rx_array_ptr = 0;                                            // Pointer to current position in array
    
    while(U1STAbits.URXDA) 
    {
        rx_array[rx_array_ptr++] = U1RXREG;                                     // Read byte
    }
    
    if (rx_array[0] != MY_ADDRESS)
    {

        if (U1STAbits.OERR == 1)                                                // Check for overrun error
        { 
            U1STAbits.OERR = 0;                                                 // Clear error flag
        } 
        else if(rx_array_ptr >= RX_ARRAY_SIZE)                                  // Check if array is full
        {
            rx_array_ptr = 0;                                                   // Reset array pointer
            for(i=0; i<RX_ARRAY_SIZE; i++) 
            {
                rx_array[i] = 0;                                                // Clear array
            }
        } 
        else if (rx_array[rx_array_ptr-1] == '\n')                              // Check for end of message
        { 
            IFS3bits.U1RXIF = 0;                                                // Clear RX interrupt flag

            rx_array[rx_array_ptr++] = U1RXREG;                                 // Read received data into array

            if (rx_array_ptr == MESSAGE_LENGTH+1)                               // Check if message is complete
            {     
                uint8_t address = rx_array[0];                                  // Get address from message
                uint8_t checksum = rx_array[CHECKSUM_INDEX];                    // Get checksum from message

                uint8_t calculated_checksum = 0;                                // Variable for calculated checksum
                for (uint8_t i = 0; i < CHECKSUM_INDEX; i++) 
                {
                    calculated_checksum ^= rx_array[i];                         // XOR each byte
                }

                if (calculated_checksum == checksum)                            // Check if checksum is correct
                { 
                    if (address == MY_ADDRESS)                                  // Check if address matches
                    { 
                        process_message(rx_array, MESSAGE_LENGTH);              // Process message
                    }
                } 
                else 
                {
                    handle_checksum_error();                                    // Handle checksum error
                    rx_array_ptr = 0;                                           // Reset array pointer
                    return;
                }
                rx_array_ptr = 0;                                               // Reset array pointer
            }
        }
    }
}

void UART_TX_Char(char c) 
{
    while (U1STAbits.UTXBF);                                                    // Wait if the transmit buffer is full
    U1TXREG = c;                                                                // Transmit the character
}

void UART_TX_String(const char *str) 
{
    while (*str != '\0') 
    {
        UART_TX_Char(*str);                                                     // Transmit each character
        str++;
    }
}

void process_message(uint8_t* message, uint8_t length) 
{     
    switch(message[1]) 
    {
        case 'V':                                                               // Voltage selection
                voltage_call(message);
                UART_TX_Char('P');
            break;
        case 'S':                                                               // Voltage selection
                shelf_call(message);
                UART_TX_Char('P');
            break;
        case 'R':                                                               // Rack selection     
                reset_call(message);
                UART_TX_Char('P');
            break;   
        case 'I':                                                               // Rack selection
                bottomTop_call(message);
                UART_TX_Char('P');
            break;   
        case 'P':                                                               // Position selection
                position_call(message);
                UART_TX_Char('P');
            break;  
        case 'E':                                                               // Enable features on backboard         
                enable_call(message);
                UART_TX_Char('P');
            break;      
        case 'A':                                                               // ADC readings
                adc_call(message);
                UART_TX_Char('P');
            break;
        default:
                UART_TX_Char('F');
            break;
    }
}

void __ISR(_EXTERNAL_1_VECTOR, IPL5AUTO) rackInsertTop(void)
{
    __builtin_disable_interrupts();
    
    IFS0bits.INT1IF = 0;                                                        // Clear the interrupt flag

    topInsert = !topInsert;                                                     // Toggle the state of topInsert
    
    INTCONbits.INT1EP = !INTCONbits.INT1EP;                                     // Toggle the edge detection mode
    
    if (topInsert)                                                              // If the interrupt is the rack being inserted 
    {                                                            
        if(!bottomInsert)                                                        // If the bottom rack is already in
        {
            LATBbits.LATB9 = 1;                                                 // EN_SET1 to 1
            LATBbits.LATB10 = 1;                                                // EN_SET2 to 1  
            
            LATBbits.LATB14 = 1;                                                // RESET_TR to 1
       
            for(i=0; i<200000000; i++) __asm volatile ("nop");   
            
            LATBbits.LATB14 = 0;                                                // RESET_TR to 0
            
            LATBbits.LATB9 = 0;                                                 // EN_SET1 to 0
            LATBbits.LATB10 = 0;                                                // EN_SET2 to 0
        }
    } 
    else 
    {
        LATBbits.LATB9 = 1;                                                     // EN_SET1 to 1
        LATBbits.LATB10 = 1;                                                    // EN_SET2 to 1    
        
        LATBbits.LATB12 = 0;                                                    // EN_VOSC_TR to 0
        LATBbits.LATB14 = 1;                                                    // RESET_TR to 1
        
        for(i=0; i<200000000; i++) __asm volatile ("nop"); 
        
        LATBbits.LATB14 = 0;                                                    // RESET_TR to 0
        
        LATBbits.LATB9 = 0;                                                     // EN_SET1 to 0
        LATBbits.LATB10 = 0;                                                    // EN_SET2 to 0
    }  
    
    __builtin_enable_interrupts();
}

void __ISR(_EXTERNAL_0_VECTOR, IPL6AUTO) rackInsertBottom(void)
{
    __builtin_disable_interrupts();
    
    IFS0bits.INT0IF = 0;                                                        // Clear the interrupt flag
    
    INTCONbits.INT0EP = !INTCONbits.INT0EP;                                     // Toggle the edge detection mode
    
    bottomInsert = !bottomInsert;                                               // Toggle the state of bottomInsert
    
    if (bottomInsert)                                                           // If the interrupt is the rack being inserted
    {    
        LATBbits.LATB9 = 1;                                                     // EN_SET1 to 1
        LATBbits.LATB10 = 1;                                                    // EN_SET2 to 1 
        
        switch (powerVariable) 
        {
            case 0:                                                             // 0V Set
                LATBbits.LATB7 = 0;                                             // S0 to 0
                LATBbits.LATB8 = 0;                                             // S1 to 0
                powerCheck = 0.0;                                               // Set known voltage comparison
                break;
            case 1:                                                             // 3.3V Set
                LATBbits.LATB7 = 1;                                             // S0 to 1
                LATBbits.LATB8 = 0;                                             // S1 to 0
                powerCheck = 0.66;                                              // Set known voltage comparison
                break;
            case 2:                                                             // 5V Set
                LATBbits.LATB7 = 0;                                             // S0 to 0
                LATBbits.LATB8 = 1;                                             // S1 to 1
                powerCheck = 1.0;                                               // Set known voltage comparison
                break;
            case 3:                                                             // 12V Set
                LATBbits.LATB7 = 1;                                             // S0 to 1
                LATBbits.LATB8 = 1;                                             // S1 to 1
                powerCheck = 2.4;                                               // Set known voltage comparison
                break;
            default:                                                            // Unknown Set
                LATBbits.LATB7 = 0;                                             // S0 to 0
                LATBbits.LATB8 = 0;                                             // S1 to 0
                powerCheck = 0.0;                                               // Set known voltage comparison
                break;
        }
        
        for(i=0; i<200000000; i++) __asm volatile ("nop"); 
        
        LATBbits.LATB9 = 0;                                                     // EN_SET1 to 0
        LATBbits.LATB10 = 0;                                                    // EN_SET2 to 0
    }
    else 
    {
        LATBbits.LATB9 = 1;                                                     // EN_SET1 to 1
        LATBbits.LATB10 = 1;                                                    // EN_SET2 to 1
        
        LATBbits.LATB15 = 1;                                                    // RESET_PWR to 1
        LATBbits.LATB13 = 1;                                                    // RESET_BR to 1                                

        LATBbits.LATB7 = 0;                                                     // S0 to 0
        LATBbits.LATB8 = 0;                                                     // S1 to 0
        LATBbits.LATB11 = 0;                                                    // EN_VOSC_BR to 0

        for(i=0; i<200000000; i++) __asm volatile ("nop");                            

        LATBbits.LATB13 = 0;                                                    // RESET_BR to 0
        LATBbits.LATB15 = 0;                                                    // RESET_PWR to 0
        
        LATBbits.LATB9 = 0;                                                     // EN_SET1 to 0
        LATBbits.LATB10 = 0;                                                    // EN_SET2 to 0
        
        LATDbits.LATD2 = 0;                                                     // Enables to 0
        LATDbits.LATD3 = 0;
        LATDbits.LATD4 = 0;
        LATDbits.LATD5 = 0;
    }
    
    __builtin_enable_interrupts();
}

double ADC_Read(uint8_t channel)
{
    if (channel >= 0 && channel <= 4) 
    {
        uint16_t result[5];
        ADCCON3bits.GSWTRG = 1;                                                 // Trigger ADC conversions
        while (ADCDSTAT1bits.ARDY0 == 0);                                       // Wait for ADC0 conversion to complete
        result[0] = ADCDATA0;                                                   // Fetch ADC0 result
        while (ADCDSTAT1bits.ARDY1 == 0);                                       // Wait for ADC1 conversion to complete
        result[1] = ADCDATA1;                                                   // Fetch ADC1 result
        while (ADCDSTAT1bits.ARDY2 == 0);                                       // Wait for ADC2 conversion to complete
        result[2] = ADCDATA2;                                                   // Fetch ADC2 result
        while (ADCDSTAT1bits.ARDY3 == 0);                                       // Wait for ADC3 conversion to complete
        result[3] = ADCDATA3;                                                   // Fetch ADC3 result
        while (ADCDSTAT1bits.ARDY4 == 0);                                       // Wait for ADC4 conversion to complete
        result[4] = ADCDATA4;                                                   // Fetch ADC4 result

        return result[channel];                                                 // Return 12-bit integer
    } 
    else 
    {
        return 0;
    }
}

void handle_checksum_error(void) 
{
    return;                                                                     // Exit the interrupt handler
}

void adc_call(uint8_t* message) 
{
    if (message[2] == 'V' && message[3] == 'O') 
    {
        adc = ADC_Read(4);
        sprintf(adcStr, "%d", adc);
        UART_TX_String(adcStr);
    } 
    else if (message[2] == 'T') 
    {
        if (message[3] == 'T') 
        {
            adc = ADC_Read(3);
            sprintf(adcStr, "%d", adc);
            UART_TX_String(adcStr);
        } 
        else if (message[3] == 'B') 
        {
            adc = ADC_Read(2);
            sprintf(adcStr, "%d", adc);
            UART_TX_String(adcStr);
        }
    } 
    else if (message[2] == 'I') 
    {
        if (message[3] == 'B') 
        {
            adc = ADC_Read(1);
            sprintf(adcStr, "%d", adc);
            UART_TX_String(adcStr);
        } 
        else if (message[3] == 'T') 
        {
            adc = ADC_Read(0);
            sprintf(adcStr, "%d", adc);
            UART_TX_String(adcStr);
        }
    }
}

void voltage_call(uint8_t* message) 
{
    if(message[2] == 'O') 
    {
        if(message[3] == '1') 
        {
            powerVariable = 1;                                                  // Set internal variable to 1
        } 
        else if(message[3] == '2') 
        {
            powerVariable = 2;                                                  // Set internal variable to 2
        } 
        else if (message[3] == '3') 
        {   
            powerVariable = 3;                                                  // Set internal variable to 3
        } 
        else 
        {
            powerVariable = 0;                                                  // Set internal variable to 0
        }
    }
}

void shelf_call(uint8_t* message) 
{
    if(message[2] == 'O') 
    {
        if(message[3] == '1') 
        {
            LATBbits.LATB9 = 1;                                                 // EN_SET1 to 1
            LATBbits.LATB10 = 1;                                                // EN_SET1 to 1

            for(i=0; i<200000000; i++) __asm volatile ("nop");                                      

            LATBbits.LATB7 = 1;                                                 // S0 to 1
            LATBbits.LATB8 = 0;                                                 // S1 to 0

            for(i=0; i<200000000; i++) __asm volatile ("nop"); 

            LATBbits.LATB9 = 0;                                                 // EN_SET1 to 0
            LATBbits.LATB10 = 0;                                                // EN_SET1 to 0
        } 
        else if(message[3] == '2') 
        {
            LATBbits.LATB9 = 1;                                                 // EN_SET1 to 1
            LATBbits.LATB10 = 1;                                                // EN_SET1 to 1

            for(i=0; i<200000000; i++) __asm volatile ("nop");                                       

            LATBbits.LATB7 = 0;                                                 // S0 to 0
            LATBbits.LATB8 = 1;                                                 // S1 to 1

            for(i=0; i<200000000; i++) __asm volatile ("nop");                                    

            LATBbits.LATB9 = 0;                                                 // EN_SET1 to 0
            LATBbits.LATB10 = 0;                                                // EN_SET1 to 0
        } 
        else if (message[3] == '3') 
        {   

            LATBbits.LATB9 = 1;                                                 // EN_SET1 to 1
            LATBbits.LATB10 = 1;                                                // EN_SET1 to 1

            for(i=0; i<200000000; i++) __asm volatile ("nop");                                      

            LATBbits.LATB7 = 1;                                                 // S0 to 1
            LATBbits.LATB8 = 1;                                                 // S1 to 1

            for(i=0; i<200000000; i++) __asm volatile ("nop");                                    

            LATBbits.LATB9 = 0;                                                 // EN_SET1 to 0
            LATBbits.LATB10 = 0;                                                // EN_SET1 to 0
        } 
        else 
        {
            LATBbits.LATB15 = 1;                                                // RESET_PWR to 1

            for(i=0; i<200000000; i++) __asm volatile ("nop");                                     

            LATBbits.LATB7 = 0;                                                 // S0 to 0
            LATBbits.LATB8 = 0;                                                 // S1 to 0

            for(i=0; i<200000000; i++) __asm volatile ("nop");                                  

            LATBbits.LATB15 = 0;                                                // RESET_PWR to 0
        }
    }
}

void reset_call(uint8_t* message) 
{
    if (message[2] == 'E') 
        {
            if (message[3] == 'B') 
            {
                bottomInsert = false;
                topInsert = false;
            }
        }
}

void bottomTop_call(uint8_t* message) 
{
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
        else if (message[3] == '0')
        {
            LATFbits.LATF0 = 0;
            LATFbits.LATF1 = 0;
            LATFbits.LATF3 = 1;
            LATFbits.LATF4 = 0;
            LATFbits.LATF5 = 0;
        }
    }
    else if (message[2] == 'T') 
    {
        if (message[3] == '1') 
        {
            LATFbits.LATF0 = 1;
            LATFbits.LATF1 = 1;
            LATFbits.LATF3 = 0;
            LATFbits.LATF4 = 0;
            LATFbits.LATF5 = 1; 
        } 
        else if (message[3] == '0')
        {
            LATFbits.LATF0 = 0;  
            LATFbits.LATF1 = 0;
            LATFbits.LATF3 = 0;
            LATFbits.LATF4 = 0; 
            LATFbits.LATF5 = 0; 
        }
    }
}
void position_call(uint8_t* message) 
{
    if (message[2] >= '0' && message[2] <= '9' 
            && message[3] >= '0' && message[3] <= '9')
    {
        int position[32][5] = 
        {
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
    }
}

void enable_call(uint8_t* message)
{
    if(message[2] == 'V')                                                       // Enable VTest
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

    if(message[2] == 'R')                                                       // Enable RF
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

    if(message[2] == 'E')                                                       // Enable EFC
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

    if(message[2] == 'M')                                                       // Enable Thermal addressing switch
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

    if(message[2] == 'B')                                                       // Enable OSC power
    { 
        LATBbits.LATB9 = 1;                                                     // EN_SET1 to 1
        LATBbits.LATB10 = 1;                                                    // EN_SET2 to 1

        if (message[3] ==  '1')
        {
            LATBbits.LATB11 = 1;                                                // EN_VOSC BR to 1
        }
        else
        {
            LATBbits.LATB13 = 1;                                                // RESET_BR to 1
            LATBbits.LATB11 = 0;                                                // EN_VOSC BR to 0
        }

        for(i=0; i<200000000; i++) __asm volatile ("nop"); 

        LATBbits.LATB13 = 0;                                                    // RESET_BR to 0

        LATBbits.LATB9 = 0;                                                     // EN_SET1 to 0
        LATBbits.LATB10 = 0;                                                    // EN_SET1 to 0

    }

    if(message[2] == 'T')   
    {
        LATBbits.LATB9 = 1;                                                     // EN_SET1 to 1
        LATBbits.LATB10 = 1;                                                    // EN_SET2 to 1

        if (message[3] ==  '1')
        {
            LATBbits.LATB12 = 1;                                                // EN_VOSC TR to 1
        }
        else
        {
            LATBbits.LATB12 = 0;                                                // EN_VOSC TR to 0
            LATBbits.LATB14 = 1;                                                // RESET_TR to 1
        }

        for(i=0; i<200000000; i++) __asm volatile ("nop"); 

        LATBbits.LATB14 = 0;                                                    // RESET_TR to 0

        LATBbits.LATB9 = 0;                                                     // EN_SET1 to 0
        LATBbits.LATB10 = 0;                                                    // EN_SET1 to 0       
    } 
}