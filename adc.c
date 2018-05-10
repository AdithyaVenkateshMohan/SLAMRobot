#include "xc.h"
#include "Graphics.h"

void initADC( int);
double returnTemp(void);

void initADC( int amask)
{
     ANSELA = amask;    // select analog input pins
    AD1CON1 = 0x00E0;   // auto convert after end of sampling
    AD1CSSL = 0;        // no scanning required
    AD1CON3 = 0x1F3F;   // max sample time = 31Tad, Tad = 2 x Tcy
    AD1CON2 = 0;        // use MUXA, AVss and AVdd are used as Vref+/-
    AD1CON1bits.ADON = 1; // turn on the ADC

    _TRISB9 = 1;        // make RB9 an input

    // Explorer 16 Development Board Errata (work around 2)
    // RB15 should always be a digital output
    _LATB15 = 0;
    _TRISB15 = 0;
} // InitADC

double returnTemp(void)
{
   // T3CON = 0x8000;  
    AD1CHS0  = 9;               // select analog input channel
 AD1CON1bits.SAMP=1; // automatic sampling after previous conversion
    double temp=0;
    while (!AD1CON1bits.DONE);
   
temp=ADC1BUF0;

temp=(3300/1023)*temp;
  temp=((temp)-500)/10;

  temp=temp;
  
return temp;
}

int compareTemp( double temp )
{ int status=0;
    if (temp>21)
    {
        status = RED;
    }
    else
    {
        if(temp < 19)
        {
        
            status = BLUE ;
        }
        else
        {
        
            status = YELLOW;
        
        }
    }
return status;
}