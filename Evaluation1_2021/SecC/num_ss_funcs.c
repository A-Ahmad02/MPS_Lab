#include "gpio_addr.h"

void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}

void GPIOinit(void)
{
	int delay_clk;
	//enable clock on portA,B,D and F
	SYSCTL_RCGC_GPIO_R |= 0x2B;
	//dummy cycles
	delay_clk=SYSCTL_RCGC_GPIO_R;
	
	//Unlock PF0
	GPIO_PORTF_LOCK_R |= 0x4C4F434B;
	GPIO_PORTF_CR_R |= PORTF_PIN0;
	
	
	//digital enable port A,B,D
	GPIO_PORTA_DEN_R |= PORTA_PINS;
	GPIO_PORTB_DEN_R |= PORTB_PINS;
	GPIO_PORTD_DEN_R |= PORTD_PINS;
	GPIO_PORTF_DEN_R |= PORTF_PIN0;
	
	//set port A, B as output and PORTD,F as input
	GPIO_PORTA_DIR_R |= PORTA_PINS;
	GPIO_PORTB_DIR_R |= PORTB_PINS;
	GPIO_PORTD_DIR_R &= (~PORTD_PINS);
	GPIO_PORTF_DIR_R &= (~PORTF_PIN0);
	
	//enable pulldown on PORTD,F
	GPIO_PORTD_PDR_R |= PORTD_PINS;
	GPIO_PORTF_PUR_R |= PORTF_PIN0;
	
}

void Interruptinit(void)
{
	DisableInterrupts();
	GPIO_PORTF_IM_R  &= 0x00;										//EnableinterruptonPF0
	GPIO_PORTF_IS_R  &= ~PORTF_PIN0;									//PF0 is edge sensitive
	GPIO_PORTF_IBE_R &= ~PORTF_PIN0;									//PF0 is not both edges
	GPIO_PORTF_IEV_R |= PORTF_PIN0;									//PF0 is rising edge
	GPIO_PORTF_ICR_R |= PORTF_PIN0;									//Clear interrupt flag for PF0
	GPIO_PORTF_IM_R  |= PORTF_PIN0;										//EnableinterruptonPF0
	
	
	NVIC_EN0_R |= 0x40000000;
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00A00000;
	EnablePriorityInterrupts();
	EnableInterrupts();
}

