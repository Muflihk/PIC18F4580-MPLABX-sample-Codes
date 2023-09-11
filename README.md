# PIC18F4580 MPLABX Sample codes
Advanced microcontrollers msc electronics
 MPLAB IDE Link
[ https://www.microchip.com/en-us/tools-resources/archives/mplab-ecosystem#MPLAB%20IDE%20Archives](https://ww1.microchip.com/downloads/en/DeviceDoc/MPLAB_IDE_8_89.zip)

### 1KHz Square wave , Duty cycle 50%
```C
#include <xc.h>

#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000


void main (void)
{
     
       TRISB=0;
       LATB = 0;

        while(1)
       {
           //  1KHZ square
           LATBbits.LB2=1;   //RB2
           __delay_us(500);  //ON time
            LATBbits.LB2=0;
            __delay_us(500); //OFF Time
  
       }

   

}

```
### LED Blinking
```C
#include <xc.h>

#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000

// LED Blink
void main (void)
{
     
       TRISB=0;
       LATB = 0;

        while(1)
       {
           
           LATBbits.LB2=1;   //LED ON 
           __delay_ms(500);  // Delay 
            LATBbits.LB2=0; //LED OFF 
            __delay_ms(500); 
  
       }

   

}
 
```
### Switch and LED 
```C
#include <xc.h>

#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000

 
 void main() {
   
    TRISBbits.TRISB2= 0; // RB2 as an output (LED)
    TRISCbits.TRISC0 = 1; // RC0 is an input  (Switch)

    // Initialize RB2 (LED) as OFF
    LATBbits.LB2 = 0;

    while (1)
    {
        // Check if the button is pressed (active low)
        if (PORTCbits.RC0 == 0) 
        {
            // Turn ON the LED
            LATBbits.LB2 = 1;
        } else 
        {
            // Turn OFF the LED
            LATBbits.LB2 = 0;
        }
    }
}
```
### Seven segment counter

```C 
 #include <xc.h>

#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000

  // Seven Segment pins connected to RB0 -RB7 (Segments a-g)
  unsigned char SelectDigits[10] =
    {
        0b00111111, // 0
        0b00000110, // 1
        0b01011011, // 2
        0b01001111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111  // 9
    };

void main()
{
     
   
    TRISB = 0; // PORTB output
    int count = 0;
    
    while (1) {
       
            __delay_ms(1000); //  delay

            count++; // Increment the count
            if (count > 9)  //if greater than 9, reset to zero
            {
                count = 0; 
            }
       
           PORTB = SelectDigits[count]; // Select digit
    }
}


```
### Seven segment counter single switch (Up Counter)
```C

#include <xc.h>

#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000

 
 // Seven Segment pins connected to RB0 -RB7 (Segments a-g)
  unsigned char SelectDigits[10] =
    {
        0b00111111, // 0
        0b00000110, // 1
        0b01011011, // 2
        0b01001111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111  // 9
    };

void main()
{
     
   
    TRISB = 0; // PORTB output
    TRISCbits.RC0=1; // Switch input
    
    int count = 0;
    
   while (1) 
    {
       
           if (PORTCbits.RC0==0) // if button pressed
           {
            __delay_ms(100); // Debounce delay
              while (PORTCbits.RC0==0); // Wait for button release
        
            count++; // Increment the count
            if (count > 9)  //if greater than 9, reset to zero
            {
                count = 0; 
            }
            
           }
       
           PORTB = SelectDigits[count];
    }
}


 

```
### Seven segment up and down counter (two switches)
```C

 #include <xc.h>

#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000

 
 // Seven Segment pins connected to RC0 -RC7 (Segments a-g)
  unsigned char SelectDigits[10] =
    {
        0b00111111, // 0
        0b00000110, // 1
        0b01011011, // 2
        0b01001111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111  // 9
    };

void main()
{
     
   
    TRISC = 0; // PORTC output
    TRISB=0XFF; // Switch input
    
    int count = 0;
    
   while (1) 
    {
       
           if (PORTBbits.RB6==0) // if button pressed
           {
               
            __delay_ms(100); // Debounce delay
              while (PORTBbits.RB6==0); // Wait for button release
        
            count++; // Increment the count
            if (count > 9)  //if greater than 9, reset to zero
            {
                count = 0; 
            }
            
           }
           
            if (PORTBbits.RB7==0) // if button pressed
           {
               
            __delay_ms(100); // Debounce delay
              while (PORTBbits.RB7==0); // Wait for button release
        
            count--; // Increment the count
            if (count < 0)  //if greater than 9, reset to zero
            {
                count = 9; 
            }
            
           }
       
           PORTC = SelectDigits[count];
    }
}

```
### LCD Interfacing 4 bit mode
```C

#include <xc.h>

#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000

  
#define RS PORTCbits.RC3
#define EN PORTCbits.RC2
#define LCD_PORT LATD   // LCD D4 -D7 PORTD 4 BI TMODE

//-----Function declaration
void LCD_Init();
void LCD_Command(unsigned char);
void LCD_Out(char );
void LCD_String(const char*);
void LCD_Clear();


void main() 
{
    TRISD=0;
    TRISC=0;
    LCD_Init();
    LCD_String("Hello, World!");

    while (1) 
    {
        
    }
}
 
 void LCD_Init() 
{
  
    LCD_Command(0x02);
    __delay_ms(10);
    LCD_Command(0x28);
    __delay_ms(10);
    LCD_Command(0x0E);
    __delay_ms(10);
    LCD_Command(0x01);
    __delay_ms(2);
    LCD_Command(0x06);
}
 void LCD_Clear()
{
    LCD_Command(0x01);
    __delay_ms(2);
}

void LCD_Command(unsigned char command) 
{
    RS = 0;
    LCD_PORT = (command >> 4) & 0x0F; // Send higher nibble 
    EN = 1;
    EN = 0;
    __delay_us(1);
    LCD_PORT = command & 0x0F; // Send lower nibble 
    EN = 1;
    EN = 0;
    __delay_us(100);
}

void LCD_Out(char data)
{
    RS = 1;
    LCD_PORT = (data >> 4) & 0x0F; // Send higher nibble
    EN = 1;
    EN = 0;
    __delay_us(1);
    LCD_PORT = data & 0x0F; // Send lower nibble 
    EN = 1;
    EN = 0;
    __delay_us(100);
}

void LCD_String(const char* data) // Send string
{
    while (*data != '\0') 
    {
        LCD_Out(*data);
        data++;
    }
}

```
### Serial data send (UART)
```C
 #include <xc.h>

#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000

  
 void UART_Init();
 void UART_Write(char);

void main() 
{
    
    
    // Initialize UART
    UART_Init();
    
    while(1) {
        // Send a character
        UART_Write('H');
        UART_Write('e');
        UART_Write('l');
        UART_Write('l');
        UART_Write('o');
        UART_Write(13); // New line
      
         __delay_ms(1000); //delay
    }
}
 // Function to initialize UART
void UART_Init() 
{
    // Set the baud rate to 9600 (configurations for 20MHz crystal)
    TRISC = 0X80;
    SPBRG =0X81;
    TXSTA = 0X24;
    RCSTA = 0X90;

}

// Function to send a character over UART
void UART_Write(char data)
{
    while(!TXIF);           // Wait until the transmitter is ready
    TXREG = data;           // Transmit data
}

```
### ADC Value to serial port

```C
 #include <xc.h>
#include <stdio.h>
#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000

 void UART_Init();
 void UART_Write(char);
 void Send_String(char *s);
 void ADC_Init(); 

void main() 
{
    
    char buffer[6];
    int ADC_Value=0;
    // Initialize UART
    UART_Init();
    ADC_Init(); //Init ADC
    
    while(1) 
    {
             
        GO_nDONE = 1;     // Start ADC conversion
        while (GO_nDONE); // Wait for conversion to complete
        ADC_Value=(ADRESH << 8) + ADRESL; // Return the 10-bit ADC result
      
        sprintf(buffer," ADC = %d ",ADC_Value); // ADC value to ASCII string
        Send_String(buffer);
        UART_Write(13); // New line
         __delay_ms(1000); //delay
    }
}
 // Function to initialize UART
void UART_Init() 
{
    // Set the baud rate to 9600 (configurations for 20MHz crystal)
    TRISC = 0X80;
    SPBRG =0X81;
    TXSTA = 0X24;
    RCSTA = 0X90;

}

// Function to send a character over UART
void UART_Write(char data)
{
    while(!TXIF);           // Wait until the transmitter is ready
    TXREG = data;           // Transmit data
}
void Send_String(char *s)
{
    while (*s++)
    {
      UART_Write(*s);
    }
}
void ADC_Init() {
    ADCON1 = 0x80; // Set AN0/RA0 as analog input, Vref- and Vref+ as VSS and VDD
    ADCON2 = 0x8C; // Right justify result, Tad = Fosc/16
    ADCON0 = 0x01; // ADC module enabled, channel AN0 selected
}


```
### Serial Volt Meter using ADC

```C
 #include <xc.h>
#include <stdio.h>
#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000


 void UART_Init();
 void UART_Write(char);
 void Send_String(char *s);
 void ADC_Init(); 

void main() 
{
    
    char buffer[15];
    int ADC_Value=0;
    float Voltage=0;
    // Initialize UART
    UART_Init();
    ADC_Init(); //Init ADC
    
    while(1) 
    {
             
        GO_nDONE = 1;     // Start ADC conversion
        while (GO_nDONE); // Wait for conversion to complete
        ADC_Value=(ADRESH << 8) + ADRESL; // Return the 10-bit ADC result
      
        Voltage =  (ADC_Value*5.0)/1024; // Convert o voltage
        sprintf(buffer," Voltage = %0.2f ",Voltage);
        Send_String(buffer);
        UART_Write(13); // New line
         __delay_ms(1000); //delay
    }
}
 // Function to initialize UART
void UART_Init() 
{
    // Set the baud rate to 9600 (configurations for 20MHz crystal)
    TRISC = 0X80;
    SPBRG =0X81;
    TXSTA = 0X24;
    RCSTA = 0X90;

}

// Function to send a character over UART
void UART_Write(char data)
{
    while(!TXIF);           // Wait until the transmitter is ready
    TXREG = data;           // Transmit data
}
void Send_String(char *s)
{
    while (*s++)
    {
      UART_Write(*s);
    }
}
void ADC_Init() {
    ADCON1 = 0x80; // Set AN0/RA0 as analog input, Vref- and Vref+ as VSS and VDD
    ADCON2 = 0x8C; // Right justify result, Tad = Fosc/16
    ADCON0 = 0x01; // ADC module enabled, channel AN0 selected
}


```
### Temperature monitoring through serial port

```C
 #include <xc.h>
#include <stdio.h>
#pragma config OSC=HS
#pragma config WDT=OFF
#pragma config PWRT=OFF
#pragma config BOREN=OFF
#pragma config LVP=OFF
#pragma config CPD=OFF

#define _XTAL_FREQ 20000000


 void UART_Init();
 void UART_Write(char);
 void Send_String(char *s);
 void ADC_Init(); 

void main() 
{
    
    char buffer[15];
    int ADC_Value=0;
    float Voltage=0;
    // Initialize UART
    UART_Init();
    ADC_Init(); //Init ADC
    
    while(1) 
    {
             
        GO_nDONE = 1;     // Start ADC conversion
        while (GO_nDONE); // Wait for conversion to complete
        ADC_Value=(ADRESH << 8) + ADRESL; // Return the 10-bit ADC result
      
        Voltage =  (ADC_Value*5.0)/1024; // Convert to voltage
        sprintf(buffer," Temperature = %0.2f",Voltage*100); // Convert to temperature
        Send_String(buffer);
        UART_Write(13); // New line
         __delay_ms(1000); //delay
    }
}
 // Function to initialize UART
void UART_Init() 
{
    // Set the baud rate to 9600 (configurations for 20MHz crystal)
    TRISC = 0X80;
    SPBRG =0X81;
    TXSTA = 0X24;
    RCSTA = 0X90;

}

// Function to send a character over UART
void UART_Write(char data)
{
    while(!TXIF);           // Wait until the transmitter is ready
    TXREG = data;           // Transmit data
}
void Send_String(char *s)
{
    while (*s++)
    {
      UART_Write(*s);
    }
}
void ADC_Init() {
    ADCON1 = 0x80; // Set AN0/RA0 as analog input, Vref- and Vref+ as VSS and VDD
    ADCON2 = 0x8C; // Right justify result, Tad = Fosc/16
    ADCON0 = 0x01; // ADC module enabled, channel AN0 selected
}



```
