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

 
 // Seven Sgment pins connected to RB0 -RB7 (Segments a-g)

void main()
{
     
    const unsigned char SevenDigits[10] =
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
    TRISB = 0; // RB7 is input for the button, others are outputs
    char count = 0;
    
    while (1) {
       
            __delay_ms(1000); //  delay

            count++; // Increment the count
            if (count > 9)  //if greater than 9, reset to zero
            {
                count = 0; 
            }
       
           PORTB = SevenDigits[count]; // Select digit
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
 // switch connected to RC0
void main()
{
     
    const unsigned char SevenDigits[10] =
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
    TRISB = 0; // RB7 is input for the button, others are outputs
    TRISCbits.RC0=1; // Switch input
    char count = 0;
    
    while (1) 
    {
       
           if (PORTCbits.RC0==0) 
           {
            __delay_ms(100); // Debounce delay
              while (PORTCbits.RC0==0); // Wait for button release
        
            count++; // Increment the count
            if (count > 9)  //if grater than 9, reset to zero
            {
                count = 0; 
            }
            
           }
       
           PORTB = SevenDigits[count];
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

 
 // Seven Segment pins connected to RB0 -RB7 (Segments a-g)
 // switch1 connected to RC0
 //  switch2 connected to RC1
void main()
{
     
    const  unsigned char SevenDigits[10] =
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
    TRISB = 0; // RB7 is input for the button, others are outputs
    TRISCbits.RC0=1;  // Switch up input
     TRISCbits.RC1=1; // Switch down input
    char count = 0;
    
    while (1) 
    {
       
           if (PORTCbits.RC0==0) // if button1 is pressed
           {
            __delay_ms(100); // Debounce delay
              while (PORTCbits.RC0==0); // Wait for button1 release
        
            count++; // Increment the count
            if (count > 9)  //if grater than 9, reset to zero
            {
                count = 0; 
            }
            
           }
           
            if (PORTCbits.RC1==0) // if button2 is pressed
           {
            __delay_ms(100); // Debounce delay
              while (PORTCbits.RC1==0); // Wait for button2 release
        
              
            if (count < 0)  //if less than 0, set to 9
             {
                count=9; 
             }
              count--; // decrement count
            
           }
       
            LATB = SevenDigits[count];
           
    }
 
 
 
 }
``` 

```
