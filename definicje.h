#ifndef __definicje_h__
#define __definicje_h__
#define RCC_AHB1ENR     (*((unsigned int*)(RCC_BASE + 0x30)))
#define RCC_APB2ENR     (*((unsigned int*)(RCC_BASE + 0x44)))

#define GPIOD_MODER     (*((unsigned int*)(GPIOD_BASE)))
#define GPIOD_SPEED     (*((unsigned int*)(GPIOD_BASE+0x08)))
#define GPIOD_PUPDR     (*((unsigned int*)(GPIOD_BASE+0x0c)))
#define GPIOD_BSRR      (*((unsigned int*)(GPIOD_BASE+0x18)))
#define GPIOD_ODR       (*((unsigned int*)(GPIOD_BASE+0x14)))

#define LED_Green_On    (1<<12)
#define LED_Orange_On   (1<<13)  
#define LED_Red_On      (1<<14)  
#define LED_Blue_On     (1<<15)  
#define LED_Green_Off   (1<<28)
#define LED_Orange_Off  (LED_Orange_On<<16)
#define LED_Red_Off     (LED_Red_On<<16)
#define LED_Blue_Off    (LED_Blue_On<<16)

#define GPIOA_MODER     (*((unsigned int*)(GPIOA_BASE)))
#define GPIOA_OTYPER    (*((unsigned int*)(GPIOA_BASE + 0x04)))
#define GPIOA_SPEED     (*((unsigned int*)(GPIOA_BASE + 0x08)))
#define GPIOA_PUPDR     (*((unsigned int*)(GPIOA_BASE + 0x0c)))
#define GPIOA_IDR       (*((unsigned int*)(GPIOA_BASE+0x10)))


#define SysTick_CTRL    (*((unsigned int*)(SysTick_BASE)))
#define SysTick_LOAD    (*((unsigned int*)(SysTick_BASE+0x04)))
#define SysTick_VAL     (*((unsigned int*)(SysTick_BASE+0x08)))

#define SYSCFG_EXTICR1  (*((unsigned int*)(SYSCFG_BASE+0x08)))
#define SYSCFG_EXTICR2  (*((unsigned int*)(SYSCFG_BASE+0x0c)))
#define SYSCFG_EXTICR3  (*((unsigned int*)(SYSCFG_BASE+0x10)))
#define SYSCFG_EXTICR4  (*((unsigned int*)(SYSCFG_BASE+0x14)))

#define EXTI_IMR       (*((unsigned int*)(EXTI_BASE)))
#define EXTI_EMR       (*((unsigned int*)(EXTI_BASE+0x04)))
#define EXTI_RTSR      (*((unsigned int*)(EXTI_BASE+0x08)))
#define EXTI_FTSR      (*((unsigned int*)(EXTI_BASE+0x0c)))
#define EXTI_SWIER     (*((unsigned int*)(EXTI_BASE+0x10)))
#define EXTI_PR        (*((unsigned int*)(EXTI_BASE+0x14)))

#define TIM1_CR1        (*((unsigned int*)(TIM1_BASE)))
#define TIM1_CR2        (*((unsigned int*)(TIM1_BASE+0x04)))
#define TIM1_SMCR       (*((unsigned int*)(TIM1_BASE+0x08)))
#define TIM1_DIER       (*((unsigned int*)(TIM1_BASE+0x0c)))
#define TIM1_SR         (*((unsigned int*)(TIM1_BASE+0x10)))
#define TIM1_EGR        (*((unsigned int*)(TIM1_BASE+0x14)))
#define TIM1_CCMR1      (*((unsigned int*)(TIM1_BASE+0x18)))
#define TIM1_CCMR2      (*((unsigned int*)(TIM1_BASE+0x1c)))
#define TIM1_CCER       (*((unsigned int*)(TIM1_BASE+0x20)))
#define TIM1_CNT        (*((unsigned int*)(TIM1_BASE+0x24)))
#define TIM1_PSC        (*((unsigned int*)(TIM1_BASE+0x28)))
#define TIM1_ARR        (*((unsigned int*)(TIM1_BASE+0x2c)))
#define TIM1_RCR        (*((unsigned int*)(TIM1_BASE+0x30)))
#define TIM1_CCR1        (*((unsigned int*)(TIM1_BASE+0x34)))
#define TIM1_CCR2        (*((unsigned int*)(TIM1_BASE+0x38)))
#define TIM1_CCR3        (*((unsigned int*)(TIM1_BASE+0x3c)))
#define TIM1_CCR4        (*((unsigned int*)(TIM1_BASE+0x40)))
#define TIM1_BDTR        (*((unsigned int*)(TIM1_BASE+0x44)))
#define TIM1_DCR        (*((unsigned int*)(TIM1_BASE+0x48)))
#define TIM1_DMAR       (*((unsigned int*)(TIM1_BASE+0x4c)))

#define ADDRESS	        GPIOD->ODR |= LED_Green_On
#define ADDRESS_E     	GPIOD->ODR &= ~(LED_Green_On)   
#define WRITE	        GPIOD->ODR |= LED_Orange_On
#define WRITE_E	        GPIOD->ODR &= ~(LED_Orange_On)
#define READ	        GPIOD->ODR |= LED_Red_On
#define READ_E	        GPIOD->ODR &= ~(LED_Red_On)
#define ACTION	        GPIOD->ODR |= LED_Blue_On
#define ACTION_E   	GPIOD->ODR &= ~(LED_Blue_On)




#endif
