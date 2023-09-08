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
    TRISCbits.TRISC0 = 1; // RB0 is an input  (Switch)

    // Initialize RB7 (LED) as OFF
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
