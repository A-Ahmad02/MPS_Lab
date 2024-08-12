// Timer 1B of 5ms with 1s LED toggle 
#include "TM4C123.h"

void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |             /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}
//constant values
#define		TIM1_CLK_EN             0x02
#define		TIM1_EN                 0x01

#define		TIM_16_BIT_EN           0x04
#define		TIM_32_BIT_EN           0x00

#define		TIM_TAMR_PERIODIC_EN    0x02
#define		TIM1_INT_CLR            0x01		// Interrupt GPTM Timer A 

#define		EN0_INT21               0x200000 // 16/32-Bit Timer 1A

#define		PORTF_CLK_EN            0x20
#define		TOGGLE_PF1              0x02
#define		LED_RED                 0x02	

//function headers
void GPIO_Init(void);
void Timer_Init(unsigned long period);
void DisableInterrupts(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);

void Timer_Init(unsigned long period)
{
	//enable clock for Timer1
	SYSCTL->RCGCTIMER |= TIM1_CLK_EN ;	// Turn on Timer 1
	
	//disable Timer1 before setup
	TIMER1->CTL  &= ~(TIM1_EN);		// Disable Timer 1							
	//configure 32-bit timer mode
	TIMER1->CFG  |= (TIM_32_BIT_EN);		// Use whole timer
	//configure periodic mode
	TIMER1->TAMR = TIM_TAMR_PERIODIC_EN;	// Enable Periodic same for TA and wholeTimer 	
	//set initial load value
	TIMER1->TAILR = period;	
	//set prescalar for desired frequency 16M
	TIMER1->TAPR  = 0;	
	
	DisableInterrupts();
	//Set priority for interrupt
	NVIC_SetPriority(TIMER1A_IRQn,2);
	//enable interrupt 21 (timer1A interrupt)												
	NVIC_EnableIRQ(TIMER1A_IRQn);
	//clear timeout interrupt
	TIMER1->ICR  |= TIM1_INT_CLR;										
	//enable interrupt mask for Timer0A
	TIMER1->IMR  |= TIM1_INT_CLR;												
	
	//enableTimer0A
	TIMER1->CTL  |= TIM1_EN;												
	
	EnableInterrupts();
}


void GPIO_Init()
{
	//configure red led (PF1)
	SYSCTL->RCGCGPIO |= PORTF_CLK_EN;
	GPIOF->DEN       |= LED_RED;
	GPIOF->DIR       |= LED_RED;
}


void TIMER1A_Handler(void)
{
	TIMER1->ICR   = TIM1_INT_CLR;
}



int main(void)
{
	//initialize PF1 as digital output
	GPIO_Init();	
		GPIOF->DATA  ^= TOGGLE_PF1;
	Timer_Init(80000000);		
	while ((TIMER1->RIS & 0x00000001) == 0);
		GPIOF->DATA  ^= TOGGLE_PF1;
	TIMER1->CTL  &= ~(TIM1_EN);		// Disable Timer 1		
	
	
	while(1)
		WaitForInterrupt();
}
