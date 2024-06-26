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
#pragma config FPLLRNG = RANGE_5_10_MHZ                                         // System PLL Input Range
#pragma config FPLLICLK = PLL_POSC                                              // System PLL Input Clock Selection = Primary Oscillator
#pragma config FPLLMULT = MUL_80                                                // System PLL Multiplier, 80 = 200MHz SYSCLK
#pragma config FPLLODIV = DIV_2                                                 // System PLL Output Clock Divider

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

int main(void)
{        
    Sys_Init();
   
    while(1){}
    return 0;
}

void Sys_Init(void)
{
    __builtin_disable_interrupts();
    
    unsigned int i = 0;
    
    for(i = 0; i < 1000000; i++){} 
    
    /* ********************************************************************** */
    /* Tristate settings
    /* ********************************************************************** */
    TRISB = 0xFF;                                                               // Port B default to all Inputs

    TRISC = 0xFF;                                                               // Port C default to all Inputs
    TRISCbits.TRISC14 = 1;                                                      // Input for 5 MHz clock

    TRISD = 0xFF;                                                               // Port D default to all Inputs
    TRISDbits.TRISD11 = 0;
    TRISDbits.TRISD10 = 0;                                                      // Input for T6CK, Prior Output for LE of SY8929 fine adjust line delay
    TRISDbits.TRISD9 = 0;                                                       // D9 of delay line parallel bus
    TRISDbits.TRISD1 = 0;                                                       // Input for T4CK
    
    TRISE = 0xFF;                                                               // Port E default to all Inputs  
    
    TRISF = 0xFF;                                                               // Port F default to all Inputs
   
    TRISG = 0xFF;                                                               // Port G default to all Inputs
    TRISGbits.TRISG6 = 1;                                                       // Output for delayed PPS out
    TRISGbits.TRISG7 = 0;                                                       // UART1 TX output
    TRISGbits.TRISG8 = 1;                                                       // UART1 RX input
    TRISGbits.TRISG9 = 0;     
    
    /* ********************************************************************** */
    /* Analog settings
    /* ********************************************************************** */
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
  
    for(i = 0; i < 1000000; i++){}                                                 // pause on boot to allow UART to stabilize  

    SYSKEY = 0xAA996655;                                                        // first unlock key
    SYSKEY = 0x556699AA;                                                        // second unlock key
    SPLLCON = 0x014F0101;                                                       // PLL set for 200MHz system clock with 10MHz clock input frequency at pin
    OSCCONbits.NOSC = 1;                                                        // set new oscillator section to 1 = Internal Fast Internal RC Oscillator with PLL module via Postscaler (FRCPLL)
    OSCCONbits.OSWEN = 1;                                                       // set oscillator switch enable to 1 = Initiate an oscillator switch to selection specified by NOSC<2:0> bits
    PB2DIVbits.PBDIV = 3;                                                       // peripheral bus clock 3 set to div by 0 for no division = SYSCLK freq, ds 717
    OSCCONCLR = 0x10;                                                           // set the power-saving mode to an idle mode
    SYSKEY = 0x33333333;                                                        // lock key
    
    PRISS = 0x76543210;                                                         // assign shadow set #7-#1 to priority level #7-#1 ISRs
    INTCONSET = _INTCON_MVEC_MASK;                                              // configure Interrupt Controller for multi-vector mode
    __builtin_enable_interrupts();                                              // set the CP0 Status register IE bit high to globally enable interrupts

    init_uart(); 
    
}

void init_uart(void)
{
    U1BRG = 53;                                                                 // set for 57600
    U1STA = 0;                                                                  // clear status control register for UART 1
    U1MODE = 0x0000;                                                            // clear UART 1 mode register
 
    U1STAbits.UTXEN = 1;                                                        // transmit is enabled
    
    U1STAbits.URXEN = 1;                                                        // receive is enabled                    
    IPC28bits.U1RXIP = UART_RX_INT_PRIORITY;                                    // set interrupt priority
    IEC3bits.U1RXIE = 1;                                                        // enable RX interrupt
    IFS3bits.U1RXIF = 0; 
    
    while (U1STAbits.URXDA) 
    {
        volatile char dummy = U1RXREG; 
    }
    
    while (!U1STAbits.TRMT);
    
    U1MODEbits.ON = 1;                                                          // turn on UART1
}