/* ************************************************************************** */
/* DEVCFG3
/* ************************************************************************** */
#pragma config FMIIEN = ON                                                      // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON                                                      // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = OFF                                                    // Permission Group Lock One Way Configuration
#pragma config PMDL1WAY = OFF                                                   // Peripheral Module Disable Configuration
#pragma config IOL1WAY = OFF                                                    // Peripheral Pin Select Configuration
#pragma config FUSBIDIO = OFF                                                   // USB off

/* ************************************************************************** */
/* DEVCFG2
/* ************************************************************************** */
#pragma config FPLLIDIV = DIV_2                                                 // System PLL Input Divider
#pragma config FPLLRNG = RANGE_8_16_MHZ                                         // System PLL Input Range
#pragma config FPLLICLK = PLL_POSC                                              // System PLL Input Clock Selection = Primary Oscillator
#pragma config FPLLMULT = MUL_80                                                // System PLL Multiplier, x80 = 400MHz 
#pragma config FPLLODIV = DIV_2                                                 // System PLL Output Clock Divider, /2 = 200MHz SYSCLK

/* ************************************************************************** */
/* DEVCFG1
/* ************************************************************************** */
#pragma config FNOSC = SPLL                                                     // Oscillator Selection Bits = system PLL
#pragma config DMTINTV = WIN_127_128                                            // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = OFF                                                    // Secondary Oscillator Enable (Enable Secondary Oscillator)
#pragma config IESO = ON                                                        // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = EC                                                     // Primary Oscillator Configuration
#pragma config OSCIOFNC = OFF                                                   // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME                                                   // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576                                                // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP                                                   // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL                                                  // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF                                                     // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25                                             // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31                                                   // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF                                                     // Deadman Timer Enable (Deadman Timer is enabled)

/* ************************************************************************** */
/* DEVCFG0
/* ************************************************************************** */
#pragma config DEBUG = OFF                                                      // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF                                                     // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1                                                // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = OFF                                                      // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32                                                 // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED                                           // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF                                                     // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL                                                  // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM                                                // Soft Master Clear Enable (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X                                               // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON                                                   // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X                                               // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = OFF                                                  // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL                                                // EJTAG Boot Enable (Normal EJTAG functionality)

/* ************************************************************************** */
/* DEVCCP0
/* ************************************************************************** */
#pragma config CP = OFF                                                         // Code Protect (Protection Disabled)

#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mz2048efh064.h>
#include <cp0defs.h> 
#include "RS485.h"
#include "main.h"

bool topInsert = false;
bool bottomInsert = false;

int main(void)
{        
    init_system();
    
    int i; 
    for(i=0; i<200000000; i++) __asm volatile ("nop"); 
    
    LATBbits.LATB13 = 0;
    LATBbits.LATB14 = 0;
    LATBbits.LATB15 = 0;
    
    while(1)
    {
         if (messageAvailable)
        {   
            process_message(messageBuf, MESSAGE_LENGTH);                        // Process message in the main loop
            messageAvailable = false;
        }
    }
    return 0;
}

void init_system(void)
{
    __builtin_disable_interrupts();                                             // set the CP0 Status register IE bit low to globally disable interrupts
    
    init_configuration();
    
    init_uart();
    
    init_adc();
    
    init_interrupt(); 
    
    __builtin_enable_interrupts();                                              // set the CP0 Status register IE bit high to globally enable interrupts

}

void init_uart(void)
{
    U1MODEbits.ON = 0;                                                          // Disable UART before configuration

    U1BRG = 26;                                                                 // Set the baud rate (115200)
    U1STA = 0;                                                                  // Clear UART status and mode registers
    U1MODE = 0;                                                                 // Clear UART status and mode registers
    
    while (U1STAbits.URXDA)                                                     // Clear any existing data in the receive buffer
    {
        volatile char dummy = U1RXREG;                                          // Clear the receive buffer
    }
    
    while (!U1STAbits.TRMT);                                                    // Wait until the transmit buffer is empty
    
    U1STAbits.UTXEN = 1;                                                        // Enable Transmit
    U1STAbits.URXEN = 1;                                                        // Enable Receive
   
    IPC28bits.U1RXIP = UART_RX_INT_PRIORITY;                                    // Set UART RX interrupt priority
    
    IEC3bits.U1RXIE = 1;                                                        // Enable UART RX interrupt
   
    IFS3bits.U1RXIF = 0;                                                        // Clear UART RX interrupt flag
    
    U1MODEbits.ON = 1;                                                          // Enable UART module
}   

void init_configuration(void)
{
    /* ********************************************************************** */
    /* Tristate settings
    /* ********************************************************************** */
    TRISB = 0xFF;                                                               // Port B default to all Inputs
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;                                                      
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB6 = 1;                                                     
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;                                                      // Input for T6CK, Prior Output for LE of SY8929 fine adjust line delay
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB10 = 0;
    TRISBbits.TRISB11 = 0;                                                      // Input for T6CK, Prior Output for LE of SY8929 fine adjust line delay
    TRISBbits.TRISB12 = 0; 
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB14 = 0;                                                      // Input for T6CK, Prior Output for LE of SY8929 fine adjust line delay
    TRISBbits.TRISB15 = 0;     
    
    TRISC = 0xFF;                                                               // Port C default to all Inputs
    TRISCbits.TRISC14 = 1;                                                      // Input for 5 MHz clock
    TRISCbits.TRISC15 = 0;
    
    TRISD = 0xFF;                                                               // Port D default to all Inputs
    TRISDbits.TRISD5 = 0;                                                       // D9 of delay line parallel bus
    TRISDbits.TRISD4 = 0;
    TRISDbits.TRISD3 = 0;                                                       // Input for T6CK, Prior Output for LE of SY8929 fine adjust line delay
    TRISDbits.TRISD2 = 0;                                                       // Set RD0 as input
    
    TRISE = 0xFF;                                                               // Port E default to all Inputs  
    TRISEbits.TRISE4 = 0;                                                       // D9 of delay line parallel bus
    TRISEbits.TRISE3 = 0;
    TRISEbits.TRISE2 = 0;                                                       // Input for T6CK, Prior Output for LE of SY8929 fine adjust line delay
    TRISEbits.TRISE1 = 0; 
    TRISEbits.TRISE0 = 0; 
    
    TRISF = 0xFF;                                                               // Port F default to all Inputs
    TRISFbits.TRISF5 = 0;                                                       // D9 of delay line parallel bus
    TRISFbits.TRISF4 = 0;
    TRISFbits.TRISF3 = 0;                                                       // Input for T6CK, Prior Output for LE of SY8929 fine adjust line delay
    TRISFbits.TRISF1 = 0; 
    TRISFbits.TRISF0 = 0; 
    
    TRISG = 0xFF;                                                               // Port G default to all Inputs
    TRISGbits.TRISG6 = 0;                                                       // Disable TX as output
    TRISGbits.TRISG7 = 0;                                                       // UART1 TX as output
    TRISGbits.TRISG8 = 1;                                                       // UART1 RX as input
    TRISGbits.TRISG9 = 0;                                                       // Disable RX as output
    
    /* ********************************************************************** */
    /* Analog settings
    /* ********************************************************************** */
    ANSELBbits.ANSB2 = 1;
    ANSELBbits.ANSB3 = 1;
    ANSELBbits.ANSB4 = 1;
    ANSELBbits.ANSB5 = 1;
    ANSELBbits.ANSB6 = 1;
    ANSELBbits.ANSB7 = 0;
    ANSELBbits.ANSB8 = 0;
    ANSELBbits.ANSB9 = 0;
    ANSELBbits.ANSB10 = 0;
    ANSELBbits.ANSB11 = 0;
    ANSELBbits.ANSB12 = 0;
    ANSELBbits.ANSB13 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB15 = 0;
    ANSELEbits.ANSE4 = 0;
    ANSELGbits.ANSG6 = 0;                                                       // G6 digital for delayed 1PPS output
    ANSELGbits.ANSG7 = 0; 
    ANSELGbits.ANSG8 = 0;                                                       // G8 digital for UART1 RX
    ANSELGbits.ANSG9 = 0; 
    
    /* ********************************************************************** */
    /* Peripheral Pin Select (PPS) settings
    /* ********************************************************************** */
    
    CFGCONbits.IOLOCK = 0;
    U1RXRbits.U1RXR = 0b0001;                                                   // set UART1 RX to Remappable Pin RPG8
    RPG7Rbits.RPG7R = 0b0001;                                                   // set UART1 TX to function 1, RPG7
    CFGCONbits.IOLOCK = 1;
    
    int i;
    for(i=0; i<200000000; i++) __asm volatile ("nop");   

    SYSKEY = 0xAA996655;                                                        // first unlock key
    SYSKEY = 0x556699AA;                                                        // second unlock key
    SPLLCON = 0x014F0101;                                                       // PLL set for 200MHz system clock with 10MHz clock input frequency at pin
    OSCCONbits.NOSC = 1;                                                        // set new oscillator section to 1 = Internal Fast Internal RC Oscillator with PLL module via Postscaler (FRCPLL)
    OSCCONbits.OSWEN = 1;                                                       // set oscillator switch enable to 1 = Initiate an oscillator switch to selection specified by NOSC<2:0> bits
    PB2DIVbits.PBDIV = 3;                                                       // peripheral bus clock 3 set to div by 0 for no division = SYSCLK freq, ds 717
    PB3DIVbits.PBDIV = 3; 
    OSCCONCLR = 0x10;                                                           // set the power-saving mode to an idle mode
    SYSKEY = 0x33333333;                                                        // lock key
    
    PRISS = 0x76543210;                                                         // assign shadow set #7-#1 to priority level #7-#1 ISRs
    INTCONSET = _INTCON_MVEC_MASK;                                              // configure Interrupt Controller for multi-vector mode
    
    LATB = 0x00; 
    LATBbits.LATB7 = 0;
    LATBbits.LATB8 = 0;
    LATBbits.LATB9 = 0;
    LATBbits.LATB10 = 0;
    LATBbits.LATB11 = 0;
    LATBbits.LATB12 = 0;
    LATBbits.LATB13 = 1;
    LATBbits.LATB14 = 1;
    LATBbits.LATB15 = 1;
    
    LATC = 0x00; 
    LATCbits.LATC15 = 0;
    
    LATD = 0x00; 
    LATDbits.LATD2 = 0;
    LATDbits.LATD3 = 0;
    LATDbits.LATD4 = 0;
    LATDbits.LATD5 = 0;
    
    LATE = 0x00; 
    LATEbits.LATE0 = 0;
    LATEbits.LATE1 = 0;
    LATEbits.LATE2 = 0;
    LATEbits.LATE3 = 0;
    LATEbits.LATE4 = 0;
    
    LATF = 0x00; 
    LATFbits.LATF0 = 0;
    LATFbits.LATF1 = 0;
    LATFbits.LATF3 = 0;
    LATFbits.LATF4 = 0;
    LATFbits.LATF5 = 0;
    
    LATG = 0x00; 
    LATGbits.LATG6 = 0;
    LATGbits.LATG9 = 0;
}

void init_adc(void)
{
    ADC0CFG = DEVADC0;
    ADC1CFG = DEVADC1;
    ADC2CFG = DEVADC2;
    ADC3CFG = DEVADC3;
    ADC4CFG = DEVADC4;
    ADC7CFG = DEVADC7;
    
    /* ********************************************************************** */
    /* Configure ADCCON1
    /* ********************************************************************** */

    ADCCON1 = 0;                                                                // No ADCCON1 features are enabled including: Stop-in-Idle, turbo, CVD mode, Fractional mode and scan trigger source.

    /* ********************************************************************** */
    /* Configure ADCCON2
    /* ********************************************************************** */
    
    ADCCON2 = 0;                                                                // Since, we are using only the Class 1 inputs, no setting is required for ADCDIV

    /* ********************************************************************** */
    /* Initialize warm up time register
    /* ********************************************************************** */

    ADCANCON = 0;
    ADCANCONbits.WKUPCLKCNT = 5;                                                // Wakeup exponent = 32 * TADx
    
    /* ********************************************************************** */
    /* Clock setting
    /* ********************************************************************** */

    ADCCON3 = 0;
    ADCCON3bits.ADCSEL = 1;                                                     // Select input clock source
    ADCCON3bits.CONCLKDIV = 0;                                                  // Control clock frequency is half of input clock PR031122
    ADCCON3bits.VREFSEL = 0;                                                    // Select AVDD and AVSS as reference source   
    
    /* ********************************************************************** */
    /* Select ADC sample time and conversion clock 
    /* ********************************************************************** */

    ADC0TIMEbits.ADCDIV = 1;                                                    // ADC0 clock frequency is half of control clock = TAD2 PR031122
    ADC0TIMEbits.SAMC = 5;                                                      // ADC0 sampling time = 5 * TAD2
    ADC0TIMEbits.SELRES = 3;                                                    // ADC0 resolution is 12 bits
    
    ADC1TIMEbits.ADCDIV = 1;                                                    // ADC1 clock frequency is half of control clock = TAD2 PR031122
    ADC1TIMEbits.SAMC = 5;                                                      // ADC1 sampling time = 5 * TAD2
    ADC1TIMEbits.SELRES = 3;                                                    // ADC1 resolution is 12 bits
    
    ADC2TIMEbits.ADCDIV = 1;                                                    // ADC2 clock frequency is half of control clock = TAD2 PR031122
    ADC2TIMEbits.SAMC = 5;                                                      // ADC2 sampling time = 5 * TAD2
    ADC2TIMEbits.SELRES = 3;                                                    // ADC2 resolution is 12 bits
    
    ADC3TIMEbits.ADCDIV = 1;                                                    // ADC3 clock frequency is half of control clock = TAD2 PR031122
    ADC3TIMEbits.SAMC = 5;                                                      // ADC3 sampling time = 5 * TAD2
    ADC3TIMEbits.SELRES = 3;                                                    // ADC3 resolution is 12 bits
    
    ADC4TIMEbits.ADCDIV = 1;                                                    // ADC4 clock frequency is half of control clock = TAD2 PR031122
    ADC4TIMEbits.SAMC = 5;                                                      // ADC4 sampling time = 5 * TAD2
    ADC4TIMEbits.SELRES = 3;                                                    // ADC4 resolution is 12 bits
   
    /* ********************************************************************** */
    /* Select analog input for ADC modules, no pre-sync trigger, not sync sampling 
    /* ********************************************************************** */

    ADCTRGMODEbits.SH0ALT = 1;                                                  // ADC0 = AN45
    ADCTRGMODEbits.SH1ALT = 1;                                                  // ADC1 = AN46
    ADCTRGMODEbits.SH2ALT = 0;                                                  // ADC2 = AN2
    ADCTRGMODEbits.SH3ALT = 0;                                                  // ADC3 = AN3
    ADCTRGMODEbits.SH4ALT = 0;                                                  // ADC4 = AN4
    
    /* ********************************************************************** */
    /* Select ADC input mode 
    /* ********************************************************************** */

    ADCIMCON1bits.SIGN0 = 0;                                                    // unsigned data format
    ADCIMCON1bits.DIFF0 = 0;  
    ADCIMCON1bits.SIGN1 = 0;                                                    // unsigned data format
    ADCIMCON1bits.DIFF1 = 0;  
    ADCIMCON1bits.SIGN2 = 0;                                                    // unsigned data format
    ADCIMCON1bits.DIFF2 = 0;                                                    // Single ended mode
    ADCIMCON1bits.SIGN3 = 0;                                                    // unsigned data format
    ADCIMCON1bits.DIFF3 = 0;  
    ADCIMCON1bits.SIGN4 = 0;                                                    // unsigned data format
    ADCIMCON1bits.DIFF4 = 0;
    
    /* ********************************************************************** */
    /* Configure ADCGIRQENx
    /* ********************************************************************** */

    ADCGIRQEN1 = 0;                                                             // No interrupts are used
    ADCGIRQEN2 = 0;

    /* ********************************************************************** */
    /* Configure ADCCSSx 
    /* ********************************************************************** */

    ADCCSS1 = 0;                                                                // No scanning is used
    ADCCSS2 = 0;

    /* ********************************************************************** */
    /* Configure ADCCMPCONx
    /* ********************************************************************** */

    ADCCMPCON1 = 0;                                                             // No digital comparators are used. Setting the ADCCMPCONx
    ADCCMPCON2 = 0;                                                             // register to '0' ensures that the comparator is disabled.
    ADCCMPCON3 = 0;                                                             // Other registers are ?don't care?.
    ADCCMPCON4 = 0;
    ADCCMPCON5 = 0;
    ADCCMPCON6 = 0;

    /* ********************************************************************** */
    /* Configure ADCFLTRx 
    /* ********************************************************************** */

    ADCFLTR1 = 0;                                                               // No oversampling filters are used.
    ADCFLTR2 = 0;
    ADCFLTR3 = 0;
    ADCFLTR4 = 0;
    ADCFLTR5 = 0;
    ADCFLTR6 = 0;
    
    /* ********************************************************************** */
    /* Set up the trigger sources 
    /* ********************************************************************** */

    ADCTRGSNSbits.LVL0 = 0;                                                     // Edge trigger
    ADCTRGSNSbits.LVL1 = 0;                                                     // Edge trigger
    ADCTRGSNSbits.LVL2 = 0;                                                     // Edge trigger
    ADCTRGSNSbits.LVL3 = 0;                                                     // Edge trigger
    ADCTRGSNSbits.LVL4 = 0;                                                     // Edge trigger
    
    ADCTRG1bits.TRGSRC0 = 1;                                                    // Set AN0 to trigger from software.
    ADCTRG1bits.TRGSRC1 = 1;                                                    // Set AN1 to trigger from software.
    ADCTRG1bits.TRGSRC2 = 1;                                                    // Set AN2 to trigger from software.
    ADCTRG1bits.TRGSRC3 = 1;                                                    // Set AN3 to trigger from software.
    ADCTRG2bits.TRGSRC4 = 1;                                                    // Set AN4 to trigger from software.
    
    ADCEIEN1 = 0;                                                               // No early interrupt
    ADCEIEN2 = 0;                                                   
    
    ADCCON1bits.ON = 1;                                                         // Turn the ADC on

    while(!ADCCON2bits.BGVRRDY);                                                // Wait until the reference voltage is ready
    while(ADCCON2bits.REFFLT);                                                  // Wait if there is a fault with the reference voltage
    
    ADCANCONbits.ANEN0 = 1;                                                     // Enable the clock to analog bias
    ADCANCONbits.ANEN1 = 1;                                                     // Enable the clock to analog bias
    ADCANCONbits.ANEN2 = 1;                                                     // Enable the clock to analog bias
    ADCANCONbits.ANEN3 = 1;                                                     // Enable the clock to analog bias
    ADCANCONbits.ANEN4 = 1;                                                     // Enable the clock to analog bias
    
    while(!ADCANCONbits.WKRDY0);                                                // Wait until ADC0 is ready
    while(!ADCANCONbits.WKRDY1);                                                // Wait until ADC1 is ready
    while(!ADCANCONbits.WKRDY2);                                                // Wait until ADC2 is ready
    while(!ADCANCONbits.WKRDY3);                                                // Wait until ADC3 is ready
    while(!ADCANCONbits.WKRDY4);                                                // Wait until ADC4 is ready
    
    ADCCON3bits.DIGEN0 = 1;                                                     // Enable ADC0
    ADCCON3bits.DIGEN1 = 1;                                                     // Enable ADC1
    ADCCON3bits.DIGEN2 = 1;                                                     // Enable ADC2
    ADCCON3bits.DIGEN3 = 1;                                                     // Enable ADC3
    ADCCON3bits.DIGEN4 = 1;                                                     // Enable ADC4
    
    ADCDATA0 = 0;
    ADCDATA1 = 0;
    ADCDATA2 = 0;
    ADCDATA3 = 0;
    ADCDATA4 = 0;
}

void init_interrupt(void)
{
    TRISDbits.TRISD0 = 1;                                                       // Set RD0 as input

    if (PORTDbits.RD0 == 1)                                                     // Set initial edge detection based on the current state of the pin for INT0
    {
        INTCONbits.INT0EP = 0;                                                  // Detect falling edge first if pin is initially high
        bottomInsert = true;
    } 
    else 
    {
        INTCONbits.INT0EP = 1;                                                  // Detect rising edge first if pin is initially low
    }

    IPC0bits.INT0IP = 6;                                                        // Set INT0 interrupt priority to 5
    IPC0bits.INT0IS = 0;                                                        // Set INT0 interrupt subpriority to 0

    IFS0bits.INT0IF = 0;                                                        // Clear the INT0 interrupt flag

    IEC0bits.INT0IE = 1;                                                        // Enable INT0 interrupt

    TRISDbits.TRISD1 = 1;                                                       // Set RD1 as input

    if (PORTDbits.RD1 == 1)                                                     // Set initial edge detection based on the current state of the pin for INT1
    {
        INTCONbits.INT1EP = 0;                                                  // Detect falling edge first if pin is initially high
        topInsert = true;
    } 
    else 
    {
        INTCONbits.INT1EP = 1;                                                  // Detect rising edge first if pin is initially low
    }

    IPC2bits.INT1IP = 5;                                                        // Set INT1 interrupt priority to 5
    IPC2bits.INT1IS = 0;                                                        // Set INT1 interrupt subpriority to 0

    IFS0bits.INT1IF = 0;                                                        // Clear the INT1 interrupt flag

    IEC0bits.INT1IE = 1;                                                        // Enable INT1 interrupt
}
