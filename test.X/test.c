/* 
 * File:   test.c
 * Author: Kameliya
 *
 * Created on May 15, 2025, 10:47 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <p24FJ256GA702.h>
#include <libpic30.h>

#pragma config FNOSC = FRCPLL
#pragma config PLLMODE = PLL4X
#pragma config IESO = OFF
#pragma config JTAGEN = OFF
#pragma config OSCIOFCN = ON
#pragma config ICS = PGD1  
#pragma config FWDTEN = OFF

#define FP 40000000
#define BAUDRATE 6900
#define BRGVAL ((FP/BAUDRATE)/16)-1

#define TEMP_CHANNEL  0   // AN0
#define SETPOINT_CHANNEL 1 // AN1
unsigned int temperature = 0;
unsigned int setpoint = 600;
//


void UART_init(void);
void UART_init_pins(void);
void UART_sendChar(char c);
void UART_sendString(const char* str);
void ADC_init(void);
void ADC_init_pins(void);
unsigned int ADC_read(unsigned char channel);
//void PWM_init(void);
void GPIO_init(void);

void UART_init_pins(void){
    TRISBbits.TRISB9 = 1;   //RXdata input 
    ANSBbits.ANSB9 = 0;     // make digital
    RPINR18bits.U1RXR = 9;  // lat1 rxdata RB9
    TRISBbits.TRISB11 = 0;  // TXdata output
    RPOR5bits.RP11R = 3;    // TXdata RB11
}


void UART_init(void){
    U1MODEbits.STSEL = 0;   //stop bit
    U1MODEbits.PDSEL = 0;   // no parity
    U1MODEbits.ABAUD = 0;   // auto baud disabled
    U1MODEbits.BRGH = 0;    // standart speed mode
    
    U1BRG = BRGVAL;
    
    U1STAbits.UTXISEL0 = 0; //interrupt after one TX
    U1STAbits.UTXISEL1 = 0; 
    
    U1STAbits.URXISEL0 = 0; //interrupt after one RX
    U1STAbits.URXISEL1 = 0;
    
    IEC0bits.U1TXIE = 1;    // enable UART TX intertupt
    IEC0bits.U1RXIE = 1;    // enable UART RX interrupt
    
    U1MODEbits.UARTEN = 1;  // enabled uart
    U1STAbits.UTXEN = 1;    // enabled uart TX
    U1STAbits.URXEN = 1;    // enable uart RX
    
    //DELAY_105uS;
    
    //U1TXREG = 'a';          // transmit one character
}


void __attribute__((__interrupt__,auto_psv)) _U1RXInterrupt(void){
    char received = U1RXREG;    //read char auto
    IFS0bits.U1RXIF = 0;    // clear rx flag
    
    int adc_value;
    char buffer[32];
    
    switch(received){
        case '1':
            adc_value = ADC_read(TEMP_CHANNEL);
            sprintf(buffer, "ADC0: %d\n", adc_value);
            UART_sendString(buffer);
            break;
        case '2':
            adc_value = ADC_read(SETPOINT_CHANNEL);
            sprintf(buffer, "ADC0: %d\n", adc_value);
            UART_sendString(buffer);
            break;
        case '3':
            temperature = ADC_read(TEMP_CHANNEL);        // AN0
            setpoint = ADC_read(SETPOINT_CHANNEL);       // AN1

            sprintf(buffer, "Temp = %d Setpoint = %d\n", temperature, setpoint);
            UART_sendString(buffer);
            break;
        case '4':
//            int new_sp = atoi(&uart_buffer[2]);
            setpoint = U1RXREG;
            sprintf(buffer, "Setpoint set to: %d\r\n", setpoint);
            UART_sendString(buffer);
           
            break;
        default:
            UART_sendString("err\n");
            break;
           
    }
}


void UART_sendChar(char c){
    while(U1STAbits.UTXBF);
    U1TXREG = c;
}

void UART_sendString(const char* str){
    while(*str){
        UART_sendChar(*str++);
    }
}

void ADC_init(void){
    //AD1PCFG = 0xFFFC;   // an0 as analog and an1 
    ANSELAbits.ANSA0 = 1;
    ANSELAbits.ANSA1 = 1;
 //   AD1PCFG = 0xFFFB;   // an2 as analog
    
    TRISBbits.TRISB0 = 1;    // RB0 input
    TRISBbits.TRISB1 = 1;   //  RB1 input
    
    AD1CON1 = 0x0000;   // samp bit = 0
    
    //AD1CHS = 0x0000;    // Channel 0 
    //AD1CHS = 0x0002;    // Connect AN2 is the input
    
    AD1CSSL = 0;
    AD1CON3 = 0x0002;   // manual saple, tad = 3tcy
    AD1CON1bits.ADON = 1;   // turn ADC ON
}

void ADC_init_pins(void){
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
}

unsigned int ADC_read(unsigned char channel){
    AD1CHS = channel; // chose channel ANx
    AD1CON1bits.SAMP = 1;   // start sampling
    //__delay_us(10);
//    DELAY_105uS
    AD1CON1bits.SAMP = 0;   // start converting
    while(!AD1CON1bits.DONE);
    return ADC1BUF0;
}

void GPIO_init(void){
    ANSAbits.ANSA2 = 0;
    TRISAbits.TRISA2 = 0;
    LATAbits.LATA2 = 1; 
}

//void PWM_init(void){
//    OC1CON1 = 0;
//    OC1CON2 = 0;
//    
//    OC1CON2bits.OCINV = 0b111;  // faza na FRC
//    OC1CON2bits.SYNCSEL = 0b111;    // sinhronizaciq
//    OC1RS = 500;
//    OC1R = 250;
//    OC1CON1bits.OCM = 0b110;
//}

/*
 * 
 */
int main(int argc, char** argv) {
    UART_init_pins();
    UART_init();
    ADC_init();
    ADC_init_pins();
    GPIO_init();
    
    while(1){
        
        if(temperature >= setpoint){
        LATAbits.LATA2 = 0;   // on
        UART_sendString("temperature is hight!\r\n");
        } else {
        LATAbits.LATA2 = 1; //off
        UART_sendString("Temperature is normal\r\n");
        }
//        if(U1STAbits.URXDA){
//            char received_char = U1RXREG;
//            while(U1STAbits.UTXBF);         
//            U1TXREG = received_char;
//        }
    }
    return (EXIT_SUCCESS);
}


