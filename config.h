

/*
 * File: PICconfig.h
 *
 * PIC24EP Mikromedia  default microcontroller configuration
 */

#include <xc.h>

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = OFF                // General Segment Code-Protect bit (General Segment Code protect is disabled)
#pragma config GSSK = OFF               // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Initial Oscillator Source Selection bits (Primary Oscillator (XT, HS, EC) with PLL)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Mode Select bits (HS Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Wait Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // Power-on Reset Timer Value Select bits (128ms)
#pragma config BOREN = ON               // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF            // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)
#pragma config ALTI2C2 = OFF            // Alternate I2C pins for I2C2 (SDA2/SCK2 pins are selected as the I/O pins for I2C2)

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config RSTPRI = PF              // Reset Target Vector Select bit (Device will obtain reset instruction from Primary flash)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FAS
#pragma config AWRP = OFF               // Auxiliary Segment Write-protect bit (Aux Flash may be written)
#pragma config APL = OFF                // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
#pragma config APLK = OFF               // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)

// Instruction Clock 40MHz
#define FCY 40000000UL     
// Defining LED's
#define LED0 _LATF0
#define LED1 _LATF1

#define Echo _RA14
#define Trigger _RA15

void IO(void){  // IO function to define the IO's as well as their initial state
    _TRISF0 = 0x0;			// defining LED's as output
    _TRISF1 = 0x0;
    _TRISA14 = 0x1;
    _TRISA15 = 0x0;
    _TRISG0 =0x0;
    _TRISG1=0x0;
    _TRISG13=0x0;
    _TRISG14=0x0;

    LED0 = 0;               // starting LED's Off
    LED1 = 0;
    Echo = 0; 
    Trigger = 0;

}
void Init(void)
{
    RCONbits.SWDTEN=0;                  // Disable Watch Dog Timer
    PLLFBD=38;				// M=40
    CLKDIVbits.PLLPOST=0;		// N1=2
    CLKDIVbits.PLLPRE=0;		// N2=2
    OSCTUN=0;				// Tune FRC oscillator, if FRC is used

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x03);	// Initiate Clock Switch to Primary
					// Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);	// Start clock switching
    while (OSCCONbits.COSC != 0b011);	// Wait for Clock switch to occur
    while(OSCCONbits.LOCK!=1) {};       // Wait for PLL to lock
}

void Delayus( int t)
{
    T1CON = 0x8000;     // enable tmr1, Tcy, 1:1
    while (t>0)         // wait for t (msec)
    {
        TMR1 = 0;
        while ( TMR1 < 40); // wait 1us
        t=t-1;
    }
} // Delayus

//void Delayms( int t)
//{
//    T1CON = 0x8000;     // enable tmr1, Tcy, 1:1
//    while (t>0)         // wait for t (msec)
//    {
//        TMR1 = 0;
//        while ( TMR1 < (FCY/1000)); // wait 1us
//        t=t-1;
//    }
//} // Delayms

