/* 
 * File:   main.c
 * Author: Aram Mine
 *
 * Created on August 17, 2024, 10:01 AM
 */

// PIC16F1827 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config DEBUG = OFF      // In-Circuit Debugger Mode (In-Circuit Debugger disabled, ICSPCLK and ICSPDAT are general purpose I/O pins)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define _XTAL_FREQ 4000000

// I2C Ack/Nack??
#define I2C_ACK  0x00
#define I2C_NACK 0xff

// ADT7410?????
#define ADT7410_I2C_WRITE_ADDRESS 0x90  // ADT7410?I2C????(Write)
#define ADT7410_I2C_READ_ADDRESS  0x91  // ADT7410?I2C????(Read)
#define ADT7410_CONFIG_ADDRESS    0x03  // ????????????????
#define ADT7410_TEMP_ADDRESS      0x00  // ????????(??8???)
#define ADT7410_CONFIG_VALUE      0x80  // ???(16-bit?????????????)

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * 
 */

/* I2C ???*/
void I2C_init(){
    SSP1CON1 = 0x28; //SSPEN = 1,I2C Master Mode
    SSP1STAT = 0x80; //???????(100KHz)
    SSP1CON3 = 0x00;
    SSP1ADD = 0x09; //Fosc/(4*Clock)-1 Clock=100kHz,Fosc=32MHz
}

// ?????????????
void i2cProtocolStart() {
    
    // SSP1CON2?????SEN????1??????
    // ?????????????????
    // ????????SSP1IF?1?????while????
    SSP1IF = 0;
	SSP1CON2bits.SEN = 1;
	while (SSP1IF == 0) {}
    SSP1IF = 0;
    
	return;
}

// ?????????????????
void i2cProtocolRepeatStart() {
    
	SSP1IF = 0;
	SSP1CON2bits.RSEN = 1;
	while (SSP1IF == 0) {}
    SSP1IF = 0;

	return;
}

// ?????????????
void i2cProtocolStop() {

    // SSP1CON2?????PEN????1??????
    // ?????????????????
    // ????????SSP1IF?1?????while????
	SSP1IF = 0;
	SSP1CON2bits.PEN = 1;
	while (SSP1IF == 0) {}
	SSP1IF = 0;

	return;
}

// 1????????
void i2cProtocolSendData(uint8_t data) {

    // SSP1BUF????????????????????????????
    // ????????SSP1IF?1?????while????
	SSP1IF = 0;
	SSP1BUF = data;
	while (SSP1IF == 0) {}
    SSP1IF = 0;
    
	return;
}

// 1????????
uint8_t i2cProtocolReceiveData() {
    
	SSP1IF = 0;
	SSP1CON2bits.RCEN = 1;
	while (SSP1IF == 0) {}
    SSP1IF = 0;

	return SSP1BUF;
}

// Ack/Nack????
uint8_t i2cProtocolCheckAck() {
    
	uint8_t ackStatus;

	if (SSP1CON2bits.ACKSTAT) {
		ackStatus = I2C_NACK;
	} else {
		ackStatus = I2C_ACK;
	}

	return ackStatus;
}

// Ack??
void i2cProtocolSendAck() {
    
    // ACKDT?ACK????(??????0???)
	SSP1CON2bits.ACKDT = 0;

    // NACK????
	SSP1CON2bits.ACKEN = 1;
	while (SSP1CON2bits.ACKEN) {}

	return;
}

// Nack??
void i2cProtocolSendNack() {
    
    // ACKDT?NACK????(??????1???)
	SSP1CON2bits.ACKDT = 1;

    // NACK????
	SSP1CON2bits.ACKEN = 1;
	while (SSP1CON2bits.ACKEN) {}

	return;
}

// ADT7410????
void adt7410Config(uint8_t config_value) {
    
    i2cProtocolStart();                              // ?????????????
    i2cProtocolSendData(ADT7410_I2C_WRITE_ADDRESS);  // ??????????
    i2cProtocolSendData(ADT7410_CONFIG_ADDRESS);     // ??????????
    i2cProtocolSendData(config_value);               // ???????
    i2cProtocolStop();                               // ?????????????
    
    return;
}

// ADT7410????
float adt7410GetTemperature(void) {
    
    uint8_t temp_high, temp_low;
    int16_t temp_value;
    
    i2cProtocolStart();                              // ?????????????
    i2cProtocolSendData(ADT7410_I2C_WRITE_ADDRESS);  // ??????????(??????)
    i2cProtocolSendData(ADT7410_TEMP_ADDRESS);       // ????????????????
    i2cProtocolRepeatStart();                        // ?????????????????
    i2cProtocolSendData(ADT7410_I2C_READ_ADDRESS);   // ??????????(??????)
    temp_high = i2cProtocolReceiveData();            // 1????????
    i2cProtocolSendAck();                            // ACK??
    temp_low  = i2cProtocolReceiveData();            // 1????????
    i2cProtocolSendNack();                           // NACK??
    i2cProtocolStop();                               // ?????????????
    
    // ????
    if( temp_high & 0x80 ) {
        temp_value = ( (temp_high & 0x7f) << 8 ) + temp_low - 32768;
    } else {
        temp_value = ( (temp_high & 0x7f) << 8 ) + temp_low;
    }

    return (float)temp_value / 128.0;
}


void lchika() {
    while(1) {
        __delay_ms(500);
        RA1 = 1;
        __delay_ms(500);
        RA2 = 1;
        __delay_ms(500);
        RA3 = 1;
        __delay_ms(500);
        RA4 = 1;
        __delay_ms(500);
        RA1 = 0;
        RA2 = 0;
        RA3 = 0;
        RA4 = 0;
    }
    return;
}

void getTempAndDisplay() {
    float temp;
    temp = adt7410GetTemperature();
    RA1 = 0;
    RA2 = 0;
    RA3 = 0;
    RA4 = 0;
    if(temp > 10) {
        RA1 = 1;
    }
    if(temp > 20) {
        RA2 = 1;
    }
    if(temp > 30) {
        RA3 = 1;
    }
    if(temp > 40) {
        RA4 = 1;
    }
    return;
}

int main(int argc, char** argv) {
    float temp;
    
    // Clock???
    OSCCON = 0b01101010;
    /*
    ????	?????
    ????	????
    ????	??????
    ????	????
    ????	????
    */
    
    // PORT???
    TRISA = 0b00000000 ; // RA?????
    TRISB = 0b00010010 ; // RB1,RB4????????
    ANSELA = 0b00000000 ; // AN0-AN4??????
    ANSELB = 0b00000000 ; // AN5-AN11??????

    PORTA = 0b00000000 ; // PORTA???
    PORTB = 0b00000000 ; // PORTB???
    
    // I2C???
    I2C_init();
    // ADT7410????????
    adt7410Config(ADT7410_CONFIG_VALUE);
    while (1) {
        getTempAndDisplay();
        __delay_ms(1000);
    }
    
    //lchika();
    
    return (EXIT_SUCCESS);
}




