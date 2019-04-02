#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"
#include "definicje.h"
#include "LSM303DLHC.h"
#include "L3GD20.h"


#define DUMMY		            0x00
#define PWM_max    (100)
#define PWM_min     (30)



void Delay(__IO uint32_t time);
void Init (void);
void PWM (unsigned int kierunek, unsigned int wartosc);
float PID (float kp, float ki, float kd, float kat, float offset);
float pomiar_odchylki (void);



extern __IO uint32_t TimmingDelay;

extern int16_t Xaxis_G,Yaxis_G;

extern float Zaxis_ms2_A, Yaxis_ms2_A, pomiar_A;
extern float Xaxis_dps_G, pomiar_G,dt_G, dt_I;
extern float pomiar;
extern float wart_wyjsc_pid, integral, derivative, proportional;
extern const uint16_t czestotliwosc;
extern uint8_t Data_A[]; // Zmienna do bezposredniego odczytu z akcelerometru
extern uint8_t Data_G[]; // Zmienna do bezposredniego odczytu z giroskopu
extern int16_t Zaxis_A,Yaxis_A; // Zawiera przeksztalcona forme odczytanych danych z akcelerometru
extern float pomiar_poprzedni, odchylka;
extern uint8_t licznik;


uint8_t tempByte;

int main()
{
  Init();
  
  
  L3GD20_write_reg(L3GD20_CTRL_REG1, PD_Normal|Data_G_rate_95Hz|X_G_Enable);
  lsm_write(LSM303DLHC_CTRL_REG1_A, Y_A_Enable| Z_A_Enable| Data_rate_100Hz);
  lsm_write(LSM303DLHC_CTRL_REG4_A, 0x08);
  lsm_read (LSM303DLHC_OUT_Y_L_A|0x80, Data_A, 4);

  //odchylka=pomiar_odchylki();
  
 
  Yaxis_A = ((Data_A[1] << 8) | Data_A[0]);
  Zaxis_A = ((Data_A[3] << 8) | Data_A[2]);
  Yaxis_ms2_A=Yaxis_A*19.62/127.9375;
  Zaxis_ms2_A=Zaxis_A*19.62/127.9375;
  pomiar_A=atanf(Yaxis_ms2_A/Zaxis_ms2_A)*57.29578;
  pomiar=pomiar_A;
 
//TIM2->PSC=28-1;//prescaler, 
//TIM2->ARR=(1000000-20000)/czestotliwosc;//auto-reload, f=1*czestotliwosc [Hz]
//TIM2->DIER=TIM_DIER_UIE;
//TIM2->CR1=TIM_CR1_CEN;

//NVIC_EnableIRQ(TIM2_IRQn);

 while(1)
 {
   
   dt_G = (0xFFFFFF - (SysTick->VAL))/6955553.7;
   if (dt_G>0.013)
   {
       SysTick->VAL   = 0xFFFFFF; 
   
       Data_G[0]=L3GD20_read_reg(L3GD20_OUT_X_L|0x80);
       Data_G[1]=L3GD20_read_reg(L3GD20_OUT_X_H|0x80);
       

       Xaxis_G = ((Data_G[1] << 8) | Data_G[0]);
       Xaxis_dps_G=(-1)*Xaxis_G/131;
       pomiar_G = pomiar+((pomiar + Xaxis_dps_G)*dt_G/2);
       
       lsm_read (LSM303DLHC_OUT_Y_L_A|0x80, Data_A, 4);
       Yaxis_A = ((Data_A[1] << 8) | Data_A[0]);
       Zaxis_A = ((Data_A[3] << 8) | Data_A[2]);
       Yaxis_ms2_A=Yaxis_A*19.62/127.9375;
       Zaxis_ms2_A=Zaxis_A*19.62/127.9375;
       pomiar_A=atanf(Yaxis_ms2_A/Zaxis_ms2_A)*57.29578;
       
      pomiar=(0.98*pomiar_G+0.02*pomiar_A);

      //wart_wyjsc_pid=PID (31, 4, 0.2, pomiar, -0.8);
      
      ///////////////////////////////////////////////////
      float kp=25+licznik*3, ki=4, kd=0.2, kat=pomiar, offset=-0.8;
       kat-=offset;
  //dt_G = (0xFFFFFF - (SysTick->VAL))/6955553.7;
   
  proportional=kat;
  derivative=Xaxis_dps_G;
          integral+=kat;
       //integral+=kat*dt_G;
       //integral+=((kat+pomiar_poprzedni)*dt_G/2);
      if (integral>((PWM_max-PWM_min)/ki))
    {
        integral=((PWM_max-PWM_min)/ki);
    }
    else if (integral<((-PWM_max+PWM_min)/ki))
    {
        integral=((-PWM_max+PWM_min)/ki);
    }
    
   
        //integral+=((kat+pomiar_poprzedni)*dt_G/2);
        
        
        pomiar_poprzedni=kat;
      
  float wartosc=kp*proportional+ki*integral+kd*derivative;
  int wartosc1=fabs(wartosc)+PWM_min;
  
  if ((kat>60)||(kat<-60.0)||((kat)>-0 && (kat<0)))
  PWM(0,0);
  else if (wartosc>=0)
    PWM(0,wartosc1);
   else
    PWM(1,wartosc1);
      //////////////////////////////////////////////////
      
      
      
      
      pomiar_poprzedni=pomiar;
   }
   
 }
  return 0;
 }


void Delay(__IO uint32_t time)
{
  TimmingDelay = time;
  while(TimmingDelay !=0);
}

void Init ()
{
RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOEEN|RCC_AHB1ENR_GPIOBEN;//wlacznie zegarow GPIO
RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN|RCC_APB2ENR_TIM1EN|RCC_APB2ENR_SPI1EN;//wlaczenie zegarow SysTick,TIM1,SPI1
RCC->APB1ENR|=RCC_APB1ENR_I2C1EN|RCC_APB1ENR_TIM2EN;//wlaczenie zegarów I2C1, TIM2

SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;
EXTI->FTSR = EXTI_FTSR_TR2;
EXTI->RTSR = EXTI_RTSR_TR0;
EXTI->IMR = EXTI_IMR_MR0 | EXTI_IMR_MR2;
NVIC_EnableIRQ(EXTI0_IRQn);


GPIOB->MODER|=0x00082000;//Pin 6,9 alternate mode
GPIOB->OTYPER|=0x0240;//Open drain
GPIOB->OSPEEDR|=0x41000;//medium speed
GPIOB->PUPDR|=0x41000;//Pull-Up 
GPIOB->AFR[0]|=0x04000000;//PB6 - I2C1_SCL
GPIOB->AFR[1]|=0x40;//PB9 - I2C1_SDA

SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR;

I2C1->CR1 = I2C_CR1_SWRST;      //Reset I2c
I2C1->CR1 = 0;                  //wylaczenie I2C na czas inicjalizacji
I2C1->CR2 = 50;                 //42MHz
I2C1->CCR = 178;                
I2C1->TRISE = 45;               
I2C1->CR1 = I2C_CR1_PE;         //Peripherial Enable

GPIOD->MODER|=(0x55<<24)|0x1;
GPIOD->OSPEEDR|=(0x55<<24)|0x1;
GPIOD->PUPDR|=(0x55<<24)|0x1;

GPIOA->MODER|=0x0012A800; //PA5, PA6, PA7 - alternative SPI1
//GPIOA->OTYPER|=0x0000;// push pull
GPIOA->OSPEEDR|=0x0013A800;//High speed, medium speed
GPIOA->PUPDR|=0x00110000;//Pull-Down
GPIOA->AFR[0]|=0x55500000;// PA5 - SCK, PA6 - MISO, PA7 - MOSI
GPIOA->AFR[1]|=0x1;

GPIOE->MODER|=0x04840040;//input, PE3 - out
//GPIOE->OTYPER|=0x00;//Push-Pull
GPIOE->OSPEEDR|=0x04c40000;//50MHZ, low speed
GPIOE->PUPDR|=0x04440040;//Pull-Down, Pull-Up
GPIOE->AFR[1]|=0x1000;

TIM1->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;//wlaczenie preload, ustawienie TIM1 jako PWM1 mode - channel 1
TIM1->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;//wlaczenie preload, ustawienie TIM1 jako PWM1 mode - channel 2
TIM1->CCER = TIM_CCER_CC1E|TIM_CCER_CC2E;//wlaczenie capture compare na chanel 1,2
TIM1->BDTR = TIM_BDTR_MOE;
TIM1->EGR = TIM_EGR_UG;
TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
TIM1->PSC = 39;//f~2kHz
TIM1->ARR = 99;

SysTick->LOAD  = (0xFFFFFF & SysTick_LOAD_RELOAD_Msk) - 1;//AHB/8 = 6,95555371875 MHz
NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
SysTick->VAL   = 0xFFFFFF;  
SysTick->CTRL|= SysTick_CTRL_ENABLE_Msk;
}

void PWM (unsigned int kierunek, unsigned int wartosc)
  {
    
     if (wartosc>=PWM_max)
     {
       TIM1->CCR1=PWM_max;
       TIM1->CCR2=PWM_max;
     }
   //else if (wartosc<=60)
     // TIM1->CCR1=60;
    else
    {
      TIM1->CCR1=wartosc+0;
      TIM1->CCR2=wartosc;
    }
  if (kierunek)
  {
    GPIOD->BSRRH=0x0001;
    GPIOE->BSRRH=0x2000;
    
    GPIOA->BSRRL=0x0400; 
    GPIOE->BSRRL=0x0200;  
   }
  else
  {
  GPIOA->BSRRH=0x0400;
  GPIOE->BSRRH=0x0200;
  
  GPIOD->BSRRL=0x0001;
  GPIOE->BSRRL=0x2000;  
  }
  }

float PID (float kp, float ki, float kd, float kat, float offset)
{
  
  kat-=offset;
  //dt_G = (0xFFFFFF - (SysTick->VAL))/6955553.7;
   
  proportional=kat;
  derivative=Xaxis_dps_G;
          integral+=kat;
       //integral+=kat*dt_G;
       //integral+=((kat+pomiar_poprzedni)*dt_G/2);
      if (integral>((PWM_max-PWM_min)/ki))
    {
        integral=((PWM_max-PWM_min)/ki);
    }
    else if (integral<((-PWM_max+PWM_min)/ki))
    {
        integral=((-PWM_max+PWM_min)/ki);
    }
    
   
        //integral+=((kat+pomiar_poprzedni)*dt_G/2);
        
        
        pomiar_poprzedni=kat;
      
  float wartosc=kp*proportional+ki*integral+kd*derivative;
  int wartosc1=fabs(wartosc)+PWM_min;
  
  if ((kat>60)||(kat<-60.0)||((kat)>-0 && (kat<0)))
  PWM(0,0);
  else if (wartosc>=0)
    PWM(0,wartosc1);
   else
    PWM(1,wartosc1);
  

 return wartosc1;
  
}

float pomiar_odchylki (void)
 {
   
    float suma_dt=0, suma=0, polozenie_zerowe=0;
    uint8_t ilosc=0;
     PWM(0,90);
   while (suma_dt<5)
   {
       dt_G = (0xFFFFFF - (SysTick->VAL))/6955553.7;
       if (dt_G>0.01)
       {
         ilosc++;
         suma_dt+=dt_G;
         SysTick->VAL   = 0xFFFFFF;
         suma_dt+=dt_G;
         Data_G[0]=L3GD20_read_reg(L3GD20_OUT_X_L|0x80);
         Data_G[1]=L3GD20_read_reg(L3GD20_OUT_X_H|0x80);
           

         Xaxis_G = ((Data_G[1] << 8) | Data_G[0]);
         Xaxis_dps_G=(-1)*Xaxis_G/131;
         pomiar_G = pomiar+((pomiar + Xaxis_dps_G)*dt_G/2);
           
         lsm_read (LSM303DLHC_OUT_Y_L_A|0x80, Data_A, 4);
         Yaxis_A = ((Data_A[1] << 8) | Data_A[0]);
         Zaxis_A = ((Data_A[3] << 8) | Data_A[2]);
         Yaxis_ms2_A=Yaxis_A*19.62/127.9375;
         Zaxis_ms2_A=Zaxis_A*19.62/127.9375;
         pomiar_A=atanf(Yaxis_ms2_A/Zaxis_ms2_A)*57.29578;
           
        pomiar=0.98*pomiar_G+0.02*pomiar_A;
        suma+=pomiar;
        polozenie_zerowe=suma/ilosc;
       }
     }
   PWM(0,0);
     //GPIOD->ODR ^= LED_Red_On;
   return polozenie_zerowe;
   
 }