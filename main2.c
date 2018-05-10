/*
 * File:   main.c
 * Author: Adithya Venkatesh Mohan, Aiswarya Balasubramanian, Ojaswi Khase
 *
 * Created on February 13, 2018, 7:21 PM
 */

#include "HardwareProfile.h"
#include "Graphics/Graphics.h"
#include "uMedia.h"
#include "xc.h"
#include "HardwareProfile.h"
#include "math.h"
#include <p24EP512GU810.h>
#include "pps.h"
#include "config.h"
#include <libpic30.h>


#define PWM_1 _RF2
#define PWM_2 _RF4
#define PWM_3 _RF8
#define LEFTA _RG13
#define RIGHTA _RG0
#define LEFTB _RG14
#define RIGHTB _RG1
#define PI 3.14159265
#define motorfwd 10
#define motorright 9
#define motorleft 6
#define motorstop 0
#define TURN 3
#define xAxis 0
#define yAxis 1
#define NEARRANGE 12
#define LEFTSERVO1 400 //left
#define LEFTSERVO2 350 //left
#define LEFTSERVO3 300 //left
#define FWDSERVO1  276 //fwd
#define FWDSERVO2 256 //fwd
#define FWDSERVO3  236 //fwd
#define RIGHTSERVO1 200 // right
#define RIGHTSERVO2 150 // right
#define RIGHTSERVO3 100 // right
#define SPEED_CMSEC 34300
#define  FORWARD 0
#define  LFT 1
#define  RGT 2
#define  NOC 12
#define TEMPMASK  0xFFFF                // make AN9 an analog input- temp sensor
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define STEP 10
#define  P0 0
#define  P1 1
#define  P2 2
#define  P3 3
#define  P4 4
#define  P5 5
#define  P6 6
#define  P7 7
#define  P8 8
#define  P9 9
int scaleDistanceforpixel(double );
void drawObstacleMAP( double, long double ultradistance[] );
unsigned long long int hexadecimalToDecimal(unsigned long int );
long double getTemp();
void distanceCompare(int);
void Init(void);
void InitScreen(void);
int getFurthestDirection(int);
void Delayus(int);
void moveServo(int);
void move( int);
void initPWMServo(void);
void DrawTurnLeftY(int leftuldist,int rightuldist,int *a,int *b, int lofline);
void DrawTurnRightY(int leftuldist,int rightuldist,int *a,int *b, int lofline);
void DrawLineObstacle(signed int uldist, int a, int b, int lofline, int axis, int colour);
void DrawLineRobot(int dist, int y);
void DrawTurnRightX(int leftuldist,int rightuldist,int *a,int *b, int lofline);
void DrawTurnLeftX(int leftuldist,int rightuldist,int *a,int *b, int lofline);
void drawtempcircle(void);
int getLeastDistance(int );
int condition=0;
long double ultradistance[9];
unsigned long long int x,y,time;
long double distanceFromUltra;
int movex=0, movey=0,originx=200,originy=160,ORIENTATION=0,pwmlength=10,distance1=10,distance2=10;

double theta=0,sei=90, currentX = SCREEN_WIDTH/2, currentY = SCREEN_HEIGHT;
 
long double startUltrasonic();
unsigned long int zero=0x0000;
unsigned long long int val;
unsigned long long int valb;
long double  computeDistance(unsigned long long int x);
long double distance[3][50];
double tempValue=0.0;
int i=0,j=0,k=0,l=0,iterate=0;
int c=0,count=0;
int current=0;
int tempstatus =0;
int leftcounter=0,rightcounter=0,stuck=0,stuckcount=0;

int main()
{   Init();
    initPWMServo();
    initADC(0xFFFF);
    IO();
   // T1CON = 0x8000;
    InitGraph();                    // init graphics library
    InitScreen();

  while(1){
              
                
               
                
                /****CHANGE P*********************/
                moveServo(LEFTSERVO1);
               
                  __delay_ms(500);
                  iterate=0;
                  count=0;
                //startUltrasonic();
               
                   // distance[LFT][i]= startUltrasonic();
                    ultradistance[P0]= startUltrasonic();
                    //dist=0;
                    
                
                Delayus(500);
                
                 /****CHANGE P*********************/
                
                
                
              moveServo(LEFTSERVO2);
                
                __delay_ms(500);
                iterate=0;
                count=0;
                //startUltrasonic();
                
                    ultradistance[P1]= startUltrasonic();
                    //dist=0;
                    
                
                  
                Delayus(500);
 
                
                 /****CHANGE P*********************/
                
                moveServo(LEFTSERVO3);
               
                 __delay_ms(500);
                 iterate=0;
                 count=0;
                //startUltrasonic();
               
                    ultradistance[P2]= startUltrasonic();
                    //dist=0;
                    
                
                Delayus(500);
                 /****CHANGE P*********************/
                 moveServo(FWDSERVO1);
               
                 __delay_ms(500);
                 iterate=0;
                 count=0;
                //startUltrasonic();
               
                    ultradistance[P3]= startUltrasonic();
                    //dist=0;
                    
                 /****CHANGE P*********************/
                     moveServo(FWDSERVO2);
               
                 __delay_ms(500);
                 iterate=0;
                 count=0;
                //startUltrasonic();
               
                    ultradistance[P4]= startUltrasonic();
                    //dist=0;
                    
                 /****CHANGE P*********************/
                     moveServo(FWDSERVO3);
               
                 __delay_ms(500);
                 iterate=0;
                 count=0;
                //startUltrasonic();
               
                    ultradistance[P5]= startUltrasonic();
                    //dist=0;
                    
                 /****CHANGE P*********************/
                     moveServo(RIGHTSERVO1);
               
                 __delay_ms(500);
                 iterate=0;
                 count=0;
                //startUltrasonic();
               
                    ultradistance[P6]= startUltrasonic();
                    //dist=0;
                    
                 /****CHANGE P*********************/
                     moveServo(RIGHTSERVO2);
               
                 __delay_ms(500);
                 iterate=0;
                 count=0;
                //startUltrasonic();
               
                    ultradistance[P7]= startUltrasonic();
                    //dist=0;
                    
                 /****CHANGE P*********************/
                
                 moveServo(RIGHTSERVO3);
               
                 __delay_ms(500);
                 iterate=0;
                 count=0;
                //startUltrasonic();
               
                    ultradistance[P8]= startUltrasonic();
                    //dist=0;
                    
                
                
                distance[LFT][i]= ( ultradistance[P0]+ ultradistance[P1]+ ultradistance[P2])/3;
                
                distance[FORWARD][i]= (ultradistance[P3]+ ultradistance[P4]+ ultradistance[P5])/3;
                
                distance[RGT][i]= (ultradistance[P6]+ ultradistance[P7]+ ultradistance[P8])/3;
               /* tempValue=getTemp();
                if (tempValue > 20)
                {
                    // set color red
                }
                
                else
                {
                    
                    //set color blue
                }*/
                distanceCompare(i);
                i++;
                //i=0;
                //d=0;
                if(i==51)
                {
                    i=0;
                }
                
    }
  
}

void InitScreen(void) {
    SetColor(WHITE);               // set background color
    DisplayBacklightOn();           // turn on the backlight
}

int getFurthestDirection(int i){
    int max=0;
    double dist=0;
    if( dist<distance[FORWARD][i]){
        max = FORWARD;
        dist=distance[FORWARD][i];
    }
    if(dist<distance[LFT][i]){
        max = LFT;
        dist=distance[LFT][i];
    }
    if(dist<distance[RGT][i]){
        max = RGT;
        dist=distance[RGT][i];
    }
    return max;
}



int getLeastDistance(int i){
    int min=NOC;
    double dist= distance[FORWARD][i];
   
    if(dist > distance[LFT][i]){
        min = LFT;
        dist=distance[LFT][i];
    }
    if(dist > distance[RGT][i]){
        min = RGT;
        dist=distance[RGT][i];
    }
    
    if (dist> NEARRANGE)
    {
        
        min = NOC;
    
    }
    return min;
}



long double computeDistance(unsigned long  long int  x)
{
    long double  distance=0,period=0;

    time=x/2;
    period= time/4000;
    period=period/1000;
    period=period/10;
    distance = SPEED_CMSEC *period;//in cm
    return distance;
}

void initPWMServo(void)
{

    _TRISF8=0x0;
    _TRISF2=0x0;
    _TRISF4= 0x0;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // ppsunlock

    RPOR11bits.RP104R = 16; //_RF8 is the port that is used PWM
    RPOR8bits.RP98R = 17;   //_RF2 is the port that is used PWM
    RPOR9bits.RP100R = 18; //_RF4 is the port that is used PWM

    __builtin_write_OSCCONL(OSCCON | 0x40); //ppslock
    OC1CON1bits.OCM     = 0;        // Output compare channel is disabled
    OC1CON1bits.OCSIDL  = 0;        // Output capture will continue to operate in CPU Idle mode
    OC1CON1bits.OCFLT   = 0;        // No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC1CON1bits.OCTSEL  = 0;        // Timer2 is the clock source for output Compare

    OC2CON1bits.OCM     = 0;        // Output compare channel is disabled
    OC2CON1bits.OCSIDL  = 0;        // Output capture will continue to operate in CPU Idle mode
    OC2CON1bits.OCFLT   = 0;        // No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC2CON1bits.OCTSEL  = 2;        // Timer4 is the clock source for output Compare
    OC2CON2bits.SYNCSEL=14;         // for sync with timer 4

    OC3CON1bits.OCM     = 0;        // Output compare channel is disabled
    OC3CON1bits.OCSIDL  = 0;        // Output capture will continue to operate in CPU Idle mode
    OC3CON1bits.OCFLT   = 0;        // No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC3CON1bits.OCTSEL  = 2;        // Timer4 is the clock source for output Compare
    OC3CON2bits.SYNCSEL=14;         // for sync with timer 4

}

void moveServo(int position)
{
    TMR2=0;
    T2CON=0x0000; //change to reset the timer before using it for
    PR2                 = 3125; // Initialize PR2 total time peroid is 20ms
    T2CON       = 0x8030;       // Start Timer2 with assumed settings
    OC1R                = position; // time peroid for duty cycle
    OC1RS               = 3125-position;    // Initialize Secondary Compare Register1 with 50% duty cycle
    OC1CON1bits.OCM     = 6;    // edge aligned
}

long double startUltrasonic()
{
    long double dist;
    while(iterate ==0 && condition <=2 ) {
        Trigger = 0;
        __delay_us(10);
        Trigger = 1;
        __delay_us(10);
        Trigger = 0;
        __delay_us(10);
        TMR3=0x0000;// set up 32 bit timer
        TMR2=0x0000;
        PR3=0xFFFF;
        PR2=0xFFFF;

        IFS0bits.T3IF=0;
        IEC0bits.T3IE=1;
        while (Echo==0);

        T2CON = 0x8008;

        while (Echo==1 && c==condition)

        {
            LED0=1;
        }

        T3CONbits.TON=0;
        T2CONbits.TON=0;

        if(c==condition)
        {

            val=hexadecimalToDecimal(TMR2);
            valb=hexadecimalToDecimal(TMR3);
            x=val+valb;
            dist= computeDistance(x);
            iterate=1;
            c=0;
            condition=0;
            __delay_ms(10);
            continue;
        }
        else
        {
            dist=1000.0;

        }
        condition++;
    }
    return dist;
}

void distanceCompare (int i)  {
    
       drawtempcircle();
    
    if(getFurthestDirection(i) == FORWARD && getLeastDistance(i)== NOC)
    {   leftcounter=0; rightcounter=0; stuck=0; stuckcount=0;
        DrawPath(i);
        drawObstacleMAP(sei,ultradistance);
        move(motorfwd);
        __delay_ms(300)
        move(motorstop);
        
    }
    else if(getFurthestDirection(i) == LFT || getLeastDistance(i)== RGT)
    {   leftcounter+=1;
    if(stuck==0)
    { stuckcount++;}
    else
    { stuckcount=0; }
    stuck=1;
    if (rightcounter > 0)
    {
        rightcounter-=1; 
    }
        theta += PI / 6;
        sei-=30;
        DrawPath(i);
        drawObstacleMAP(sei,ultradistance);
        move(motorleft);
        __delay_ms(300)
        move(motorstop);
    }
    else if (getFurthestDirection(i) == RGT || getLeastDistance(i)== LFT)
    {
        rightcounter+=1;
         if(stuck==1)
    { stuckcount++;}
        else
    { stuckcount=0; }
        stuck=0;
        if (leftcounter > 0)
        {
            leftcounter-=1;
        
        }
        theta -= PI / 6;
        sei+=30;
        
        DrawPath(i);
        drawObstacleMAP(sei,ultradistance);
        move(motorright);
        __delay_ms(300)
        move(motorstop);
    }
       
       if (stuckcount > 3)
       {  stuckcount=0; 
        move(motorfwd);
       }
}

unsigned long long int hexadecimalToDecimal( unsigned long int d)
{
    unsigned long long int decimal_number=0, remainder=0;
    while(d>zero)
    {
        remainder = d% 16;
        decimal_number = decimal_number + remainder * pow(16, count);
        d= d/ 16;
        count++;
    }
    return decimal_number;
}

void move( int value)
{
    controlDCmotor(value);
    RIGHTB= ((value & 0x001) == 0x001);
    RIGHTA=((value & 0x002) == 0x002) ;
    LEFTB=((value & 0x004) == 0x004);
    LEFTA= ((value & 0x008) == 0x008);
}

void _ISR __attribute__((interrupt,no_auto_psv)) _T3Interrupt( void)
{
    IFS0bits.T3IF=0;// clear timer interrupt
    c=c+1;
}

void controlDCmotor(int status)
{
    long unsigned int RightDutyCycle,LeftDutyCycle;

    if (status == motorleft ||status == motorright ) {
        RightDutyCycle = 800;  //50% dutycycle high speed turn
        LeftDutyCycle  = 800;
    }

    if(status == motorfwd ) {
        RightDutyCycle = 800;     //33% dutycycle lower speed for forward
        LeftDutyCycle  = 800;
    }

    if(status == motorstop) {
        RightDutyCycle = 0;     //33% dutycycle lower speed for forward
        LeftDutyCycle  = 0;
    }


    PR4                 = 3125; // Initialize PR2 total time peroid is 20ms
    T4CON               = 0x8030;       // Start Timer2 with assumed settings
    OC2R                = RightDutyCycle ;  // time peroid for duty cycle
    OC2RS               = 3125-RightDutyCycle;  // Initialize Secondary Compare Register1 with 50% duty cycle
    OC2CON1bits.OCM     = 6;    // single shot starts high by default and compares with ocr and pulls it low
    OC3R                = LeftDutyCycle;    // time peroid for duty cycle
    OC3RS               = 3125 - LeftDutyCycle; // Initialize Secondary Compare Register1 with 50% duty cycle
    OC3CON1bits.OCM     = 6;    // single shot starts high by default and compares with ocr and pulls it low
}

void DrawPath( int i){
    
    double rDist=25;
        double lDist=25;
    double newX = currentX - STEP*sin(theta);
    double newY = currentY - STEP*cos(theta);
   
    double rx=currentX - rDist*cos(theta);
    double ry=currentY + rDist*sin(theta);
    
    double lx=currentX + lDist*cos(theta);
    double ly=currentY - lDist*sin(theta);
    
    double newrx=newX - rDist*cos(theta);
    double newry=newY + rDist*sin(theta);
    
    double newlx=newX + lDist*cos(theta);
    double newly=newY - lDist*sin(theta);
    
//    double newox = currentox - STEP*sin(theta);
//    double newoy = currentoy - STEP*cos(theta);

    

 
    SetLineThickness(THICK_LINE);
    
    
    SetColor(WHITE);
    MoveTo(currentX,currentY);
    LineTo(newX,newY);
    
    SetLineType(DASHED_LINE);
     SetColor(RED);
    MoveTo(rx,ry);
    LineTo(newrx,newry);
    SetColor(RED);
    MoveTo(lx,ly);
    LineTo(newlx,newly);

    currentX = newX;
    currentY = newY;

    
}
int scaleDistanceforpixel(double distance)
{ int scaled=10;
 if (distance < 10)
    {
        scaled=5;
    
    }
    if(10<distance <20)
    {
    scaled =7;
    }

if( 20 < distance < 40)
{
    scaled =15;
}

if (40 < distance < 70)
{
    scaled =20;
}
if (70 < distance < 100)
{
scaled =25;
}

if (distance>100)
{
scaled=30;
}
return scaled;
}
void drawObstacleMAP( double sei, long double ultradistance[] )
{
    
SetColor( GREEN );  
int res=10;
double intial=sei+90;
double final = sei+90+res;
int distancelevel1=scaleDistanceforpixel(ultradistance[P0]);  
int distancelevel2=scaleDistanceforpixel(ultradistance[P1]);
int distancelevel3=scaleDistanceforpixel(ultradistance[P2]);
int distancelevel4=scaleDistanceforpixel(ultradistance[P3]);
int distancelevel5=scaleDistanceforpixel(ultradistance[P4]);
int distancelevel6=scaleDistanceforpixel(ultradistance[P5]);
int distancelevel7=scaleDistanceforpixel(ultradistance[P6]);
int distancelevel8=scaleDistanceforpixel(ultradistance[P7]);
int distancelevel9=scaleDistanceforpixel(ultradistance[P8]);

DrawArc(currentX+10,currentY,distancelevel1-2,distancelevel1,intial,final);
intial = final;
final = final +res;
DrawArc(currentX+10,currentY,distancelevel2-2,distancelevel2,sei+90,sei+90+res);
intial = final;
final = final +res;
DrawArc(currentX+10,currentY,distancelevel3-2,distancelevel3,sei+90,sei+90+res);
intial = final;
final = final +res;
DrawArc(currentX+10,currentY,distancelevel4-2,distancelevel4,sei+90,sei+90+res);
intial = final;
final = final +res;
DrawArc(currentX+10,currentY,distancelevel5-2,distancelevel5,sei+90,sei+90+res);
intial = final;
final = final +res;
DrawArc(currentX+10,currentY,distancelevel6-2,distancelevel6,sei+90,sei+90+res);
intial = final;
final = final +res;
DrawArc(currentX+10,currentY,distancelevel7-2,distancelevel7,sei+90,sei+90+res);
intial = final;
final = final +res;
DrawArc(currentX+10,currentY,distancelevel8-2,distancelevel8,sei+90,sei+90+res);
intial = final;
final = final +res;
DrawArc(currentX+10,currentY,distancelevel9-2,distancelevel9,sei+90,sei+90+res);
intial = final;
final = final +res;


}

void drawtempcircle(void)
{

        tempValue=0.0;
        tempValue=returnTemp();
        SetColor(compareTemp(tempValue));
        FillCircle(currentX, currentY,5);
        FillCircle(currentX, currentY,5);

}
