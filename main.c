#ifndef _NSC_MAIN_H
#define _NSC_MAIN_H

#include <stdint.h>

/* Standard STM32L1xxx driver headers */
#include "stm32l1xx.h"

/* STM32L1xx Discovery Kit:
    - USER Pushbutton: connected to PA0 (GPIO Port A, PIN 0), CLK RCC_AHBENR_GPIOAEN
    - RESET Pushbutton: connected RESET
    - GREEN LED: connected to PB7 (GPIO Port B, PIN 7), CLK RCC_AHBENR_GPIOBEN 
    - BLUE LED: connected to PB6 (GPIO Port B, PIN 6), CLK RCC_AHBENR_GPIOBEN
    - Linear touch sensor/touchkeys: PA6, PA7 (group 2),  PC4, PC5 (group 9),  PB0, PB1 (group 3)
*/

/* stm32l1.h  line 815. GPIO_TypeDef pointer to GPIO_BASE address 0x40020400 */
#define GPIOB               ((GPIO_TypeDef *) 0x40020400)
//#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define RCC_AHBENR_GPIOBEN  ((uint32_t)0x00000002)        /*!< GPIO port B clock enable */
#define RCC_AHBENR_GPIOAEN  ((uint32_t)0x00000001)        /*!< GPIO port A clock enable */
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)	/* line 794. EXTI pointer to this structure */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)	/* line 814 */
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define SYSCFG_EXTICR1_EXTI0  ((uint16_t)0x000F) /*!< EXTI 0 configuration line 3301 */
#define SYSCFG_EXTICR1_EXTI0_PA 	((uint16_t)0x0000) /*!< PA[0] pin line 3309 */
#define  TIM_CCMR1_CC1S     ((uint16_t)0x0003)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) line 3761 */
#define  TIM_CCMR1_CC1S_0   ((uint16_t)0x0001)            /*!<Bit 0 line 3762 */
#define  TIM_CCMR1_IC1F     ((uint16_t)0x00F0)            /*!<IC1F[3:0] bits (Input Capture 1 Filter) line 3795 */
#define  TIM_CCMR1_IC1PSC   ((uint16_t)0x000C)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) line 3791 */
#define  TIM_CCER_CC1E      ((uint16_t)0x0001)            /*!<Capture/Compare 1 output enable line 3863 */
#define  TIM_DIER_CC1IE     ((uint16_t)0x0002)            /*!<Capture/Compare 1 interrupt enable LINE 3721 */
#define  TIM_DIER_CC1DE     ((uint16_t)0x0200)            /*!<Capture/Compare 1 DMA request enable line 3729 */
volatile uint32_t current_value;
volatile uint32_t time_interval;
volatile uint32_t old_value;
int value = 0;

#endif	// Prevent the file from being included more than once.

void GPIO_Clock_Enable(){
		// Enable GPIO port B clock
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	//RCC->AHBENR	|= 0x00000002;
}

void GPIO_Init_PortB(){
		// Set pin 6 as general-purpose output
		GPIOB->MODER &= ~(0x03<<(2*6));		// Mode mask
		GPIOB->MODER |= 0x02<<(2*6);			// Set pin 6 as digital output
	
		// Set output type as push-pull
		GPIOB->OTYPER &= ~(1<<6);
	
		// Set pin 6 as alternative function 2 (TIM4)
		GPIOB->AFR[0] |= 0x2 << (4*6);
	
		// Set IO output speed
		GPIOB->OSPEEDR &= ~(0x03<<(2*6));
		GPIOB->OSPEEDR |= 0x03<<(2*6);
	
		// Set IO no pull-up pull-down
		GPIOB->PUPDR &= ~(0x00<<(2*6));	
}

void TIM4_Clock_Enable(){
		// Enable TIM4 clock
		RCC->APB1ENR	|= RCC_APB1ENR_TIM4EN;
}

/* &= bitwise AND and | |= bitwise OR logical operators */
void TIM4_Init(){
		TIM4->PSC		= 127;									// Prescaler value
		//TIM4->ARR		= 1000;								// Auto-reload value not needed coz we do input capture
		//TIM4->CCR1	= 500;								// Compare and output register coz input capture
		TIM4->CCMR1	&= ~TIM_CCMR1_CC1S;			// 0b11(0x3): CC1 channel is configured as input, IC1 is mapped on TRC
		TIM4->CCMR1	|= TIM_CCMR1_CC1S_0;		// Select bit 0
		TIM4->CCMR1	&= ~TIM_CCMR1_IC1F;			// disable digital filter
		TIM4->CCER	|= (1<<1 | 1<<3);				// CC1 channel configured as input. (11): noninverted/both rising and falling edges
		TIM4->CCMR1	&= ~TIM_CCMR1_IC1PSC;		// program input capture 1 prescaler
		TIM4->CCER	|= TIM_CCER_CC1E;				// Enable capture counter
		TIM4->DIER  |= TIM_DIER_CC1IE;			// Enable interrupt request for channel 1
		TIM4->DIER  |= TIM_DIER_CC1DE;			// Enable DMA request for channel 1
		TIM4->CR1		|= TIM_CR1_CEN;					// Enable timer 4	
		NVIC_SetPriority(TIM4_IRQn, 0x03);	// Set EXTI0 priority 3(low priority)
		NVIC_EnableIRQ(TIM4_IRQn);					// Enable EXTI0 interrupt
}	

void TIM4_IRQ_handler(void){
		if(TIM4->SR & TIM_SR_CC1IF != 0){								// do if update flag is not 0 or not set
				current_value = TIM4->CCR1;									// read CCR1 clears CC1IF interrupt flag
				time_interval = current_value - old_value;	// if counting up
				old_value = current_value;
		}
}

int main(void){
		GPIO_Clock_Enable();			// Clock for led(PB.6)
		TIM4_Clock_Enable();
		GPIO_Init_PortB();
		TIM4_Init();

		// Infinite loop. Timer - Input Capture
		while(1);
}
