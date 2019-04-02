#include "stm32f4xx.h"
#include "PWM.h"

void PWM (unsigned int kierunek, unsigned int wartosc)
  {
     if (wartosc>=100)
       TIM1->CCR1=100;
   //else if (wartosc<=60)
     // TIM1->CCR1=60;
    else
      TIM1->CCR1=wartosc;
  //TIM1->CCR2=wartosc;
  if (kierunek)
  {
    GPIOD->BSRRH=0x0001;
    GPIOE->BSRRH=0x2000;
    
    GPIOA->BSRRL=0x0400; 
    GPIOE->BSRRL=0x0800;  
   }
  else
  {
  GPIOA->BSRRH=0x0400;
  GPIOE->BSRRH=0x0800;
  
  GPIOD->BSRRL=0x0001;
  GPIOE->BSRRL=0x2000;  
  }
  }