#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config  WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include "myadc.h"
#include "my_ser.h"
#include "lcd_x8.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define _XTAL_FREQ   4000000UL 
int cc = 0; //commandCounter
char cmd[10];
char setcmd[][10] = {
    "<Rt>",
    "<RA0>", "<RA1>", "<RA2>",
    "<RH>", "<RC>",
    "<RD>",
    "<WHV0>", "<WHV1>",
    "<WCV0>", "<WCV1>",
    "<W7V0>", "<W7V1>",
    "<W6V0>", "<W6V1>",
    "<W5V0>", "<W5V1>",
    "<W4V0>", "<W4V1>",
    "<W3V0>", "<W3V1>",
    "<W2V0>", "<W2V1>",
    "<W1V0>", "<W1V1>",
    "<W0V0>", "<W0V1>"
};

float sec = 0;
float min = 00.0;
float hour = 00.0;
char buff[100];
int top = 0;
int mood = 0;
const int setup0 = 0;
float voltagef;
float voltage;
float voltage1;
float raw;
float voltage;


void delay_ms(unsigned int n) {
    int x;
    for (x = 0; x < n; x++) {
        __delaywdt_ms(1);
    }
}

#define STARTVALUE  59286

void reloadTimer0(void) {
    uint16_t startvalue = STARTVALUE;
    TMR1H = startvalue >> 8;
    TMR1L = startvalue & 0xFF;
}


void EXT_Int2_isr(void) {
    delay_ms(250);
    INTCON3bits.INT2IF = 0;
    if (mood >= 2) {
        mood = 0;
    } else {
        mood++;
    } //0-s 1-m 2-h
}

void EXT_Int1_isr(void) {
    delay_ms(250);
    INTCON3bits.INT1IF = 0;
    top ^= 1; // use XOR to toggle top between 0 and 1
}

void Timer0_isr() {
    INTCONbits.TMR0IF = 0;
    if (top == 1) {
        PORTCbits.RC1 = 1;
        sec++;
        if (sec >= 60) {
            sec = 0;
            min++;
            if (min >= 60) {
                min = 0;
                hour++;
                if (hour >= 24) {
                    hour = 0;
                }
            }
        }
        reloadTimer0();
    }
}

char x;
int flagre;
int count = 0;

void RX_isr(void) {
    x = RCREG;
    if (x == '<' || x == '>') {
        delay_ms(100);
    }
    if (x == '<') {
        cmd[0] = x;
        for (int i = 1; i < 10; i++) {
            cmd[i] = 0;// cmd[<,0,0,0,....,0]
        }
        cc = 1;
     
    } else if (cc < 10) {
        cmd[cc++] = x;//<Rt>
    }
    for (int i = 0; i < 27; i++) {
        flagre = strcmp(setcmd[i], cmd);
        if (!flagre) {
            switch (i) {
                case 0:
                    sprintf(buff, "Time:%2.0f:%2.0f:%2.0f ", hour, min, sec);
                    send_string_no_lib(buff);
                    delay_ms(1000);
                    break;
                case 1:
                    raw = read_adc_raw_no_lib((unsigned char) 0);
                    voltage = raw * 5 / 1023.0;
                    sprintf(buff, "V%d = %f v, raw =%d\r", 1, voltage, raw);
                    delay_ms(500);
                    send_string_no_lib(buff);
                    break;
                case 2:
                    raw = read_adc_raw_no_lib((unsigned char) 1);
                    voltage = raw * 5 / 1023.0;
                    sprintf(buff, "V%d = %f v, raw =%d\r", 1, voltage, raw);
                    delay_ms(200);
                    send_string_no_lib(buff);
                    delay_ms(200);
                    break;
                case 3:
                    raw = read_adc_raw_no_lib((unsigned char) 2);
                    voltage = raw * 5 / 1023.0;
                    sprintf(buff, "V%d = %f v, raw =%d\r", 1, voltage * 100, raw);
                    delay_ms(200);
                    send_string_no_lib(buff);
                    break;
                case 4:
                    if (PORTCbits.RC5 == 1) {

                        sprintf(buff, "Heater: ON ");
                        send_string_no_lib(buff);
                    } else {
                        sprintf(buff, "Heater: Off ");
                        send_string_no_lib(buff);
                    }
                    break;
                case 5:
                    if (PORTCbits.RC2 == 1) {

                        sprintf(buff, "Cooler: ON ");
                        send_string_no_lib(buff);
                    } else {
                        sprintf(buff, "Cooler: Off ");
                        send_string_no_lib(buff);
                    }
                    break;
                case 6:
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 7:
                    PORTCbits.RC5 = 0;
                    break;
                case 8:
                    PORTCbits.RC5 = 1;
                    break;
                case 9:
                    PORTCbits.RC2 = 0;
                    break;
                case 10:
                    PORTCbits.RC2 = 1;
                    break;
                case 11:
                    PORTDbits.RD7 = 0;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 12:
                    PORTDbits.RD7 = 1;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 13:
                    PORTDbits.RD6 = 0;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 14:
                    PORTDbits.RD6 = 1;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 15:
                    PORTDbits.RD5 = 0;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 16:
                    PORTDbits.RD5 = 1;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 17:
                    PORTDbits.RD4 = 0;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 18:
                    PORTDbits.RD4 = 1;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 19:
                    PORTDbits.RD3 = 0;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 20:
                    PORTDbits.RD3 = 1;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 21:
                    PORTDbits.RD2 = 0;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 22:
                    PORTDbits.RD2 = 1;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 23:
                    PORTDbits.RD1 = 0;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 24:
                    PORTDbits.RD1 = 1;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 25:
                    PORTDbits.RD0 = 0;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
                case 26:
                    PORTDbits.RD0 = 1;
                    sprintf(buff, "RD7=%d RD6=%d RD5=%d RD4=%d RD3=%d RD2=%d RD1=%d RD0=%d ", PORTDbits.RD7, PORTDbits.RD6, PORTDbits.RD5, PORTDbits.RD4, PORTDbits.RD3, PORTDbits.RD2, PORTDbits.RD1, PORTDbits.RD0);
                    send_string_no_lib(buff);
                    break;
            }
            break;
        } else {
            PORTDbits.RD0 = 0;
        }
    }
}

void setupPorts(void) {
    ADCON0 = 0x00;
    ADCON1 = 0b00001100; //3 analog input
    TRISB = 0b11111111; // all pushbuttons are inputs
    TRISC = 0;
    TRISCbits.TRISC7 = 1; // RX input
    TRISCbits.TRISC6 = 0; // TX output
    PORTC = 0b00000000;
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

void init_adc_no_lib(void) {
    ADCON0 = 0;
    ADCON0bits.ADON = 1; // turn adc on 
    ADCON2 = 0b10001001; // ADFM= 1 right justified 10 bits, 2 Tad, Fosc/8
}

void __interrupt(high_priority) highIsr(void) {
    if (PIR1bits.RCIF) RX_isr();
    if (INTCONbits.TMR0IF) Timer0_isr();
    else if (INTCON3bits.INT1IF)EXT_Int1_isr();
    else if (INTCON3bits.INT2IF)EXT_Int2_isr();
    
}

void main(void) {
    setupPorts();
    init_adc_no_lib();
    INTCON = 0; // disable interrupts first, then enable the ones u want
    INTCON = 0;
    RCONbits.IPEN = 0;
    INTCONbits.INT0IE = 0;
    INTCONbits.TMR0IE = 1;
    INTCON2 = 0;
    INTCON3 = 0;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT2IE = 1;
    INTCON2bits.INTEDG1 = 1;
    INTCON2bits.INTEDG2 = 1;
    T0CON = 0x83;
    PORTD = 0xFF;
    PIE1 = 0b00100000;
    PIR1 = 0;
    IPR1 = 0;
    PIE2 = 0;
    PIE2 = 0;
    PIR2 = 0;
    IPR2 = 0;

    lcd_init();
    lcd_putc('\f');//clears the screen

    char *Cool = "OFF";
    char *Hot = "OFF";
    INTCONbits.GIEH = 1; //enable global interrupt bits
    INTCONbits.GIEL = 1;
    unsigned char dummy;
    BAUDCONbits.BRG16 = 0;
    TXSTA = 0;
    SPBRG = 25;
    SPBRGH = 0;
    TXSTAbits.BRGH = 1; //baud rate high speed option
    TXSTAbits.TXEN = 1; //enable transmission

    RCSTA = 0; // ;SERIAL RECEPTION W/ 8 BITS,
    RCSTAbits.CREN = 1; // reception
    RCSTAbits.SPEN = 1;

    dummy = RCREG; //, W        ; clear the receiver buffer      
    dummy = RCREG; //,W    
    lcd_init();
    lcd_send_byte(0, 1);
    mood = 0;

    char Buffer[32];
    char Buffer1[32];
    char Buffer2[32];
    char Buffer3[32];
    char Buffer4[32];
    char Buffer5[32];
    char Buffer6[32];

    lcd_gotoxy(1, 1);
    sprintf(Buffer, "T%2.0f:%2.0f:%2.0f %4.2fC", hour, min, sec, voltagef * 100);
    lcd_puts(Buffer);

    while (1 == 1) {
        CLRWDT();
        voltagef = read_adc_voltage(2);
        voltage = voltagef * 100.0;
        voltage1 = read_adc_voltage(1);
        lcd_gotoxy(1, 4);
        sprintf(Buffer1, "Taima Areen%2.2fV", voltage1);
        lcd_puts(Buffer1);

        if (top == setup0) {
            lcd_gotoxy(1, 1);
            sprintf(Buffer2, "T%2.0f:%2.0f:%2.0f|%4.2fC", hour, min, sec, voltagef * 100);
            lcd_puts(Buffer2);

            lcd_gotoxy(1, 3);
            if (mood == 0)
                sprintf(Buffer3, "Setup     S");
            else if (mood == 1)
                sprintf(Buffer3, "Setup     M");
            else if (mood == 2)
                sprintf(Buffer3, "Setup     H");
            lcd_puts(Buffer3);

            if (!PORTBbits.RB4) {
                delay_ms(250);
               if(mood==0){
                      if(sec == 00){
                        sec = 59;                      
                      }
                      else if(sec > 0) { 
                      sec--;                     
                      }   
                }
                 else if(mood==1){
                    if(min == 0){
                        min =59;          
                    }
                    else if(min > 0 ) { 
                      min--;
                    }
                }
                else if(mood==2){
                     if(hour >0 ) {
                         hour --;
                     }
                   else if(hour == 0) { 
                      hour =12;
                     }
                 }                              
            }
            if (!PORTBbits.RB3) {
                delay_ms(250);
               
                if(mood==0){ 
                     if(sec >= 59) { 
                      sec = 0; 
                      min++;
                     } 
                     else if(sec < 59){
                         sec++;
                     }                    
                }
                else if(mood==1){
                    if(min >= 59) { 
                      min = 0;
                      hour++;
                    }
                     else if(min < 59){
                         min++;
                     }
                }                
                   else if(mood==2){
                     if(hour >= 12) { 
                      hour = 1;
                      //sec = min =0;
                     }
                     else if(hour <12 ){ 
                         hour++;
                     }                     
                 }                              
            }
        } else {
            lcd_gotoxy(1, 1);
            sprintf(Buffer5, "T%2.0f:%2.0f:%2.0f|%4.2fC", hour, min, sec, voltagef * 100);
            lcd_puts(Buffer5);
            
            Cool = (PORTCbits.RC2 == 1) ? "ON" : "OFF";
            Hot = (PORTCbits.RC5 == 1) ? "ON" : "OFF";

            lcd_gotoxy(1, 2);
            sprintf(Buffer6, "H:%s C:%s ",Hot,Cool);
            lcd_puts(Buffer6);

            lcd_gotoxy(1, 3);
            sprintf(Buffer4, "NORMAL      ");
            lcd_puts(Buffer4);
        }    
    }
    return;
}

