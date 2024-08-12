// Timer 5ms with 1s LED PWM
// 
#include "TM4C123.h"

void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |             /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}

#define GPIO_PORTF_CR_R				(*((volatile unsigned long*)0x40025524))

//constant values
#define		TIM1_EN                 0x01		// 0th bit
#define		TIM2_EN                 0x100   // 8th bit 

#define		TIM_16_BIT_EN           0x04
#define		TIM_32_BIT_EN           0x00

//#define		TIM1_INT_CLR            0x01		// Interrupt GPTM Timer A 
//#define		TIM2_INT_CLR            0x100		// Interrupt GPTM Timer B
#define		TIM2_INT_CLR            0x400		// Interrupt GPTM Timer B Capture Mode Event Raw Interrupt

#define		PORTF_CLK_EN            0x20
#define		TOGGLE_PF1              0x02
#define		TOGGLE_PF3              0x08

#define		LED_RED                 0x02	// PF1
#define		LED_GREEN               0x08	// PF3

#define		PERIOD              		40000
#define		LEN              				200

int count;

//function headers
void GPIO_Init(void);
void Timer0_Init(void);
void Timer1_Init(void);
void DisableInterrupts(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);

void Timer1_Init(void)
{
	int delay;
	//enable clock for Timer1
	SYSCTL->RCGCGPIO |= 0x20 ;	// Turn on Port F
	delay = SYSCTL->RCGCGPIO;
	SYSCTL->RCGCTIMER |= 0x02 ;	// Turn on Timer 1
	delay = SYSCTL->RCGCTIMER;
	
	// PF3->T1CCP1
	GPIOF->DEN       |= 0x08; // 00_1000
	GPIOF->AFSEL     |= 0x08; 
	GPIOF->PCTL      |= 0x007000; 
	GPIOF->PDR |= 0x08;		// Enable weak pulldown for PF3
	
	//disable Timer1 before setup
	TIMER1->CTL  &= ~(TIM2_EN);		// Disable Timer 1	
	
	//configure 16-bit timer mode
	TIMER1->CFG  |= (TIM_16_BIT_EN);		// Use 16-bit split timer
	//configure periodic mode
	TIMER1->TBMR |= 0x20A;	// 10_0000_1010 Enable Periodic, pwm, PWM Interrupt Enable
	TIMER1->TBMR &= ~0x04;	// _0100 edge count mode 	
	//set initial load value
	TIMER1->TBILR = PERIOD;	
	//set prescalar for desired frequency 16M
	TIMER1->TBPR  = 1;	
	// Configure PWM duty cycle value ( should be less than 80000)
	TIMER1->TBMATCHR = 40000; // 75% duty cycle as ILR/4
	
	DisableInterrupts();
	//Set priority for interrupt
	NVIC_SetPriority(TIMER1B_IRQn,2);
	//enable interrupt 21 (timer1A interrupt)												
	NVIC_EnableIRQ(TIMER1B_IRQn);
	//clear timeout interrupt
	TIMER1->ICR  |= TIM2_INT_CLR;										
	//enable interrupt mask for Timer0A
	TIMER1->IMR  |= TIM2_INT_CLR;												
	
	//enableTimer0A
	TIMER1->CTL  |= TIM2_EN;												
	
	EnableInterrupts();
}
void Timer0_Init(void)
{
	int delay;
	//enable clock for Timer1
	SYSCTL->RCGCGPIO |= 0x20 ;	// Turn on Port F
	delay = SYSCTL->RCGCGPIO;
	SYSCTL->RCGCTIMER |= 0x01 ;	// Turn on Timer 0
	delay = SYSCTL->RCGCGPIO;
	
	// PF1->T0CCP1
	GPIOF->DEN       |= 0x02; // 00_0010
	GPIOF->AFSEL     |= 0x02; 
	GPIOF->PCTL      |= 0x000070; 
	GPIOF->PDR |= 0x02;		// Enable weak pulldown for PF1
		
	//disable Timer1 before setup
	TIMER0->CTL  &= ~(TIM2_EN);		// Disable Timer 1	
	
	//configure 16-bit timer mode
	TIMER0->CFG  |= (TIM_16_BIT_EN);		// Use 16-bit split timer
	//configure periodic mode
	TIMER0->TBMR |= 0x20A;	// _1010 Enable Periodic, edge count, pwm 	
	TIMER0->TBMR &= ~0x04;	// _0100 edge count mode 	
	//set initial load value
	TIMER0->TBILR = PERIOD;	
	//set prescalar for desired frequency 16M
	TIMER0->TBPR  = 1;	
	// Configure PWM duty cycle value ( should be less than 16000)
	TIMER0->TBMATCHR = 40000; // 75% duty cycle as ILR/4
	
	DisableInterrupts();
	//Set priority for interrupt
	NVIC_SetPriority(TIMER0B_IRQn,2);
	//enable interrupt 										
	NVIC_EnableIRQ(TIMER0B_IRQn);
	//clear timeout interrupt
	TIMER0->ICR  |= TIM2_INT_CLR;										
	//enable interrupt mask for Timer0A
	TIMER0->IMR  |= TIM2_INT_CLR;												
	
	//enableTimer0A
	TIMER0->CTL  |= TIM2_EN;	
	
	EnableInterrupts();
}

void GPIO_Init()
{
	//configure red led (PF1)
	SYSCTL->RCGCGPIO |= PORTF_CLK_EN;
	
	// Unlocking CR to enable PD7 alternate functionality (U2Tx)
	GPIOF->LOCK = 0x4C4F434B;	// Unlock CR 
	GPIO_PORTF_CR_R |= 0x01;				// 00_0001 (lower 5 bits) Unlock write protection
	
	//Enable user switch SW1 (PF4) and SW2 (PF0)
	GPIOF->DEN       |= 0x11;  // 01_0001
	GPIOF->DIR       &= ~0x11;  // 01_0001  input (=0)
	
	GPIOF->PUR |= 0x11;		// Enable weak pullup for PF0 and PF4
	
	
	/* INTERRUPT Configuration */
	/* Globally disable all interrupts */
	DisableInterrupts();
	
	//Set priority for interrupt
	NVIC_SetPriority(GPIOF_IRQn,1);
	//enable interrupt 										
	NVIC_EnableIRQ(GPIOF_IRQn);
	
	/* Disable all interrupts on PortF */
	GPIOF->IM &= 0x00;
	
	/* Configure PF0and4 as edge sensitive,
	* not both edges,
	* rising edge */
	GPIOF->IS &= (~0x11); 	// Set as 0 for PF4 as edge sensitive
	GPIOF->IBE &= (~0x11); // Set as 0 for PF4 as single edge sensitive
	GPIOF->IEV &= ~0x11; // Set as 0 for PF4 as falling edge sensitive
	
	/* Clear interrupt flag for PF4 */
	GPIOF->ICR |= 0x11;		// Set as 1 to clear	
	
	/* Enable interrupt on PF4 */
	GPIOF->IM |= 0x11;	

	/* Globally enable all interrupts */
	EnableInterrupts();
}


void TIMER1B_Handler(void)
{
	TIMER1->TBMATCHR = ((PERIOD-1)*count/LEN); // duty cycle as ILR/4
	count += 1;
	if (count == LEN){
		count = 0;
	}

	TIMER1->ICR  = TIM2_INT_CLR;
}

void TIMER0B_Handler(void)
{
	TIMER0->TBMATCHR = ((PERIOD-1)*(LEN-count))/LEN; // 75% duty cycle as ILR/4
	count += 1;
	if (count == LEN){
		count = 0;
	}

	TIMER0->ICR   = TIM2_INT_CLR;
}

/* Interrupt Service Routine for PortF */
void GPIOF_Handler(void){

		/* Clear interrupt flag for PF4 */
		if ((GPIOF->MIS & 0x01) == 0x01){ //PF0
			TIMER1->CTL  &= ~(TIM2_EN);		// Disable Timer 1
			GPIOF->PCTL  &= ~0x001000; 
			GPIOF->DATA &= ~(0x0F);
			GPIOF->PCTL  |= 0x000070; 
			TIMER0->CTL  |= TIM2_EN;		// Enable Timer 0
		}
		else if ((GPIOF->MIS & 0x10) == 0x10){ // PF4
			TIMER0->CTL  &= ~(TIM2_EN);		// Disable Timer 0	
			GPIOF->PCTL &= ~0x000010; 
			GPIOF->DATA &= ~(0x0F);
			GPIOF->PCTL |= 0x007000; 
			TIMER1->CTL  |= TIM2_EN;		// Enable Timer 1	
		} 
		GPIOF->ICR |= 0x11;
	
}

int main(void)
{
	Timer0_Init();
	Timer1_Init();					
	
	GPIO_Init();	
	while(1);
        //WaitForInterrupt();
}
