// PIC16F886 Configuration Bit Settings
// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = ON       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#define _XTAL_FREQ 20000000 //Crystal Oscillator Frequency


#define MAX_COUNT 3
#define MODE_CLOCK 1
#define MODE_TEMPERATURE 2


void displayClock() 
{
    RB0 = 0;
    RB1 = 0; 
    RB3 = 0; 
    RB2 = 1;
    //put display code for clock here
}

void displayTemperature()
{
    RB0 = 0;
    RB1 = 0;
    RB2 = 0; 
    RB3 = 1; 
    //put display to lcd code for temperature here
}

void LEDControl()
{
    //put LED control code here
}
 
void interface(){
    
    TRISB = 0b00100000; //Set trise bit otherwise, will not reset after press
    ANSEL = 0;
    ANSELH = 0;
    PORTB = 0b00100000; //init PORTB as outputs except RB5
    TRISE = 0x0;
    
    while(1){
        
        int displayMode = 0;
        LEDControl(); 
        while(displayMode == 0)
        {
            RB0 = 0;
            RB1 = 0;
            RB2 = 0;
            RB3 = 0; 
            if (RE3 == 0)
            {
                RB0 = 1;
            }
            else if(RE3 == 1)
            {
                RB0 = 0;
            }
            if (RB5 == 0)
            {
                RB1 = 1;
            }
            else if(RB5==1)
            {
                RB1 = 0; 
            }  
            if(RE3 == 0 && RB5 == 0)
            {
                while(RE3== 0 && RB5 == 0);

                displayMode = displayMode+1; 

            }
        }
        while(displayMode == MODE_CLOCK)
        {
            displayClock(); 
            if(RE3 == 0 && RB5 == 0)
            {
                while(RE3== 0 && RB5 == 0);
                displayMode = displayMode+1; 

            }
        }
        while(displayMode == MODE_TEMPERATURE)
        {
            displayTemperature();
            if(RE3 == 0 && RB5 == 0)
            {
                while(RE3== 0 && RB5 == 0);
                displayMode = displayMode+1; 
            }
        }
        
       
        
    }
    
}

void main(void){ 

    interface();

    return;
}