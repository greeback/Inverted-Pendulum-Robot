#include "stm32f4xx.h"
#include "PID.h"
#include "PWM.h"
#include <math.h>

float PID (float kp, float ki, float kd, float kat, float predkosc, float poprzedni, float dt)
{
//kat+=0.3;
  //dt_G = (0xFFFFFF - (SysTick->VAL))/6955553.7;
  float proportional=0;
  float derivative=0;
  float integral=0;
  
  proportional=kat;
  derivative=predkosc;
  poprzedni=kat;
      
    if ((integral>integral_max) && (derivative>0))
    {
        integral+=0;
    }
    else if ((integral<-integral_max) && (derivative<0))
    {
        integral+=0;
    }
    else 
        integral+=((kat+integral)*dt/2);
   
  float wartosc=kp*proportional+ki*integral+kd*derivative;
  int wartosc1=fabs(wartosc)+0;
  
  if ((kat>60)||(kat<-60.0)||((kat)>-0 && (kat<0)))
  PWM(0,0);
  else if (wartosc>=0)
    PWM(0,wartosc1);
   else
    PWM(1,wartosc1);
  

 return wartosc1;
  
}