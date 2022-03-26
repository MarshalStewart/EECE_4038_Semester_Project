/*
 * File:   main.c
 * Author: ponti
 *
 * Created on March 2, 2022, 9:13 AM
 */
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//---[ I2C Routines ]---

/* File: I2C_LCD.h */
 
#define _XTAL_FREQ            400000
 
#define I2C_BaudRate          100000
#define SCL_D                 TRISC3
#define SDA_D                 TRISC4
 
#define LCD_BACKLIGHT         0x08
#define LCD_NOBACKLIGHT       0x00
#define LCD_FIRST_ROW         0x80
#define LCD_SECOND_ROW        0xC0
#define LCD_THIRD_ROW         0x94
#define LCD_FOURTH_ROW        0xD4
#define LCD_CLEAR             0x01
#define LCD_RETURN_HOME       0x02
#define LCD_ENTRY_MODE_SET    0x04
#define LCD_CURSOR_OFF        0x0C
#define LCD_UNDERLINE_ON      0x0E
#define LCD_BLINK_CURSOR_ON   0x0F
#define LCD_MOVE_CURSOR_LEFT  0x10
#define LCD_MOVE_CURSOR_RIGHT 0x14
#define LCD_TURN_ON           0x0C
#define LCD_TURN_OFF          0x08
#define LCD_SHIFT_LEFT        0x18
#define LCD_SHIFT_RIGHT       0x1E
#define LCD_TYPE              2 // 0 -> 5x7 | 1 -> 5x10 | 2 -> 2 lines
 
//-----------[ Functions' Prototypes ]--------------
 
//---[ I2C Routines ]---
 
void I2C_Master_Init();
void I2C_Master_Wait();
void I2C_Master_Start();
void I2C_Master_RepeatedStart();
void I2C_Master_Stop();
void I2C_ACK();
void I2C_NACK();
unsigned char I2C_Master_Write(unsigned char data);
unsigned char I2C_Read_Byte(void);
 
//---[ LCD Routines ]---
 
void LCD_Init(unsigned char I2C_Add);
void IO_Expander_Write(unsigned char Data);
void LCD_Write_4Bit(unsigned char Nibble);
void LCD_CMD(unsigned char CMD);
void LCD_Set_Cursor(unsigned char ROW, unsigned char COL);
void LCD_Write_Char(char);
void LCD_Write_String(char*);
void Backlight();
void noBacklight();
void LCD_SR();
void LCD_SL();
void LCD_Clear();

#include <xc.h>
#include <pic16f886.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

unsigned char RS, i2c_add, BackLight_State = LCD_BACKLIGHT;

unsigned short seconds = 0;
unsigned short minutes = 0;
unsigned short hour = 0;
unsigned short day = 0;
unsigned short date = 0;
unsigned short month = 0;
unsigned short year = 0;
 
//---------------[ I2C Routines ]-------------------
//--------------------------------------------------
void I2C_Master_Init()
{
  SSPCON = 0x28;
  SSPCON2 = 0x00;
  SSPSTAT = 0x00;
  SSPADD = ((_XTAL_FREQ/4)/I2C_BaudRate) - 1;
  SCL_D = 1;
  SDA_D = 1;
}
 
void I2C_Master_Wait()
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}
 
void I2C_Master_Start()
{
  I2C_Master_Wait();
  SEN = 1;
}
 
void I2C_Master_RepeatedStart()
{
  I2C_Master_Wait();
  RSEN = 1;
}
 
void I2C_Master_Stop()
{
  I2C_Master_Wait();
  PEN = 1;
}
 
void I2C_ACK(void)
{
  ACKDT = 0; // 0 -> ACK
  I2C_Master_Wait();
  ACKEN = 1; // Send ACK
}
 
void I2C_NACK(void)
{
  ACKDT = 1; // 1 -> NACK
  I2C_Master_Wait();
  ACKEN = 1; // Send NACK
}
 
unsigned char I2C_Master_Write(unsigned char data)
{
  I2C_Master_Wait();
  SSPBUF = data;
  while(!SSPIF); // Wait Until Completion
  SSPIF = 0;
  return ACKSTAT;
}
 
unsigned char I2C_Read_Byte(void)
{
  //---[ Receive & Return A Byte ]---
  I2C_Master_Wait();
  RCEN = 1; // Enable & Start Reception
  while(!SSPIF); // Wait Until Completion
  SSPIF = 0; // Clear The Interrupt Flag Bit
  I2C_Master_Wait();
  return SSPBUF; // Return The Received Byte
}
//======================================================
 
//---------------[ LCD Routines ]----------------
//-----------------------------------------------
 
void LCD_Init(unsigned char I2C_Add)
{
  i2c_add = I2C_Add;
  IO_Expander_Write(0x00);
  __delay_ms(30);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(LCD_RETURN_HOME);
  __delay_ms(5);
  LCD_CMD(0x20 | (LCD_TYPE << 2));
  __delay_ms(50);
  LCD_CMD(LCD_TURN_ON);
  __delay_ms(50);
  LCD_CMD(LCD_CLEAR);
  __delay_ms(50);
  LCD_CMD(LCD_ENTRY_MODE_SET | LCD_RETURN_HOME);
  __delay_ms(50);
}
 
void IO_Expander_Write(unsigned char Data)
{
  I2C_Master_Start();
  I2C_Master_Write(i2c_add);
  I2C_Master_Write(Data | BackLight_State);
  I2C_Master_Stop();
}
 
void LCD_Write_4Bit(unsigned char Nibble)
{
  // Get The RS Value To LSB OF Data
  Nibble |= RS;
  IO_Expander_Write(Nibble | 0x04);
  IO_Expander_Write(Nibble & 0xFB);
  __delay_us(50);
}
 
void LCD_CMD(unsigned char CMD)
{
  RS = 0; // Command Register Select
  LCD_Write_4Bit(CMD & 0xF0);
  LCD_Write_4Bit((CMD << 4) & 0xF0);
}
 
void LCD_Write_Char(char Data)
{
  RS = 1; // Data Register Select
  LCD_Write_4Bit(Data & 0xF0);
  LCD_Write_4Bit((Data << 4) & 0xF0);
}
 
void LCD_Write_String(char* Str)
{
  for(int i=0; Str[i]!='\0'; i++)
    LCD_Write_Char(Str[i]);
}
 
void LCD_Set_Cursor(unsigned char ROW, unsigned char COL)
{
  switch(ROW)
  {
    case 2:
      LCD_CMD(0xC0 + COL-1);
      break;
    case 3:
      LCD_CMD(0x94 + COL-1);
      break;
    case 4:
      LCD_CMD(0xD4 + COL-1);
      break;
    // Case 1
    default:
      LCD_CMD(0x80 + COL-1);
  }
}
 
void Backlight()
{
  BackLight_State = LCD_BACKLIGHT;
  IO_Expander_Write(0);
}
 
void noBacklight()
{
  BackLight_State = LCD_NOBACKLIGHT;
  IO_Expander_Write(0);
}
 
void LCD_SL()
{
  LCD_CMD(0x18);
  __delay_us(40);
}
 
void LCD_SR()
{
  LCD_CMD(0x1C);
  __delay_us(40);
}
 
void LCD_Clear()
{
  LCD_CMD(0x01);
  __delay_us(40);
}

void RTC_write_time(char seconds,char minutes,char hour,char day,char date,char month, char year)
{
    I2C_Master_Start();          /*start condition */
    
    I2C_Master_Write(0xD0);     /* slave address with write mode */
    I2C_Master_Write(0x00);     /* address of seconds register written to the pointer */ 
    
    I2C_Master_Write(seconds);  /*time register values */ 
    I2C_Master_Write(minutes);
    I2C_Master_Write(hour);
    
    I2C_Master_Write(day);      /*date registers */
    I2C_Master_Write(date);
    I2C_Master_Write(month);
    I2C_Master_Write(year);
    
    I2C_Master_Stop();          /*i2c stop condition */
}

void read_time()
{
    I2C_Master_Start();          /*start condition */
    
    I2C_Master_Write(0xD0);     /* slave address with write mode */
    I2C_Master_Write(0x00);     /* address of seconds register written to the pointer */
    
    I2C_Master_RepeatedStart();

    I2C_Master_Write(0xD1);     /* slave address with read mode */
    
    seconds = I2C_Read_Byte();
    minutes = I2C_Read_Byte();
    hour = I2C_Read_Byte();
    day = I2C_Read_Byte();
    date = I2C_Read_Byte();
    month = I2C_Read_Byte();
    year = I2C_Read_Byte();
    
   /*Two things should be taken care of when reading the time
   the received values will be in BCD format and  after reading 
   the year registera non acknowledgement should be sent instead 
   of an acknowledgement */
   
    I2C_Master_Stop();          /*i2c stop condition */
}
 
 
void main(void) {
 
//  nRBPU = 0;            //Enables PORTB internal pull up resistors
//  TRISB = 0xFF;         //PORTB as input
//  TRISD = 0x00;         //PORTD as output
//  PORTD = 0x00;         //All LEDs OFF
//  I2C_Slave_Init(0x30); //Initialize as a I2C Slave with address 0x30
    
  I2C_Master_Init();
  LCD_Init(0x4E); // Initialize LCD module with I2C address = 0x4E
 
  RTC_write_time(0, 0, 0, 26, 0, 3, 2022);  

  Backlight();
//  noBacklight();
  
  while(1)
  {

    __delay_ms(1000);
    read_time();
    
    char buffer[16];
    sprintf(buffer, "%d:%d:%d", hour, minutes, seconds);
    LCD_Set_Cursor(1, 1);
    LCD_Write_String(buffer);
 
    sprintf(buffer, "%d-%d-%d", day, month, year);
    LCD_Set_Cursor(2, 1);
    LCD_Write_String(buffer);

  }
  return;
}
