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
#define		TIM0_CLK_EN             0x01
#define		TIM0_EN                 0x01
#define		TIM_16_BIT_EN           0x04
#define		TIM_TAMR_PERIODIC_EN    0x02
#define		TIM_FREQ_10usec          159
#define		TIM0_INT_CLR            0x01
#define		EN0_INT19               0x80000
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
	//enable clock for Timer0
	SYSCTL->RCGCTIMER |= TIM0_CLK_EN;	
	
	//disable Timer0 before setup
	TIMER0->CTL  &= ~(TIM0_EN);											
	//configure 16-bit timer mode
	TIMER0->CFG  |= TIM_16_BIT_EN;									
	//configure periodic mode
	TIMER0->TAMR |= TIM_TAMR_PERIODIC_EN;					
	//set initial load value
	TIMER0->TAILR = period;	
	//set prescalaer for desired frequency 100kHz
	TIMER0->TAPR  = TIM_FREQ_10usec;	
	
	DisableInterrupts();
	//Set priority for interrupt
	NVIC_SetPriority(TIMER0A_IRQn,2);
	//enable interrupt 19 (timer0A interrupt)												
	NVIC_EnableIRQ(TIMER0A_IRQn);
	//clear timeout interrupt
	TIMER0->ICR  = TIM0_INT_CLR;										
	//enable interrupt mask for Timer0A
	TIMER0->IMR  |= TIM0_EN;												
	
	//enableTimer0A
	TIMER0->CTL  |= TIM0_EN;												
	
	EnableInterrupts();
}


void GPIO_Init()
{
	//configure red led (PF1)
	SYSCTL->RCGCGPIO |= PORTF_CLK_EN;
	GPIOF->DEN       |= LED_RED;
	GPIOF->DIR       |= LED_RED;
}


void TIMER0A_Handler(void)
{
	TIMER0->ICR   = TIM0_INT_CLR;
	GPIOF->DATA  ^= TOGGLE_PF1;
}



int main(void)
{
	//generate asquare wave for 2Hz
	Timer_Init(50000);					
	//initialize PF1 as digital output
	GPIO_Init();								
	while(1)
		WaitForInterrupt();
}
