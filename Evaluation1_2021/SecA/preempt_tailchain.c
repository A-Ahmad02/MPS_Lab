#include "interrupts.h"
#include "uart0.h"
#include "gpio.h"

int high_intr, low_intr;

int main (void)
{
	UARTInit();
	InterruptInit();
	GPIOinit();
	UARTTxString("Preemption and tailchaining demonstration \r \n");
	while(1)
	{
		WaitForInterrupt();
	}
}

//port B has lower priority than port E
void GPIOB_Handler(void){	
	int i;	
	/* Clear interrupt flag for PB2 */
	GPIO_PORTB_ICR_R |= 0x04;
	low_intr = 1;
	/* Turn on the LED */
	GPIO_PORTF_DATA_R |= LED_ON ;
	/* Delay for a bit. */
	for ( i = 0 ; i < DELAY ; i++) ;
	
	if (high_intr == 1)
	{
		UARTTxString("Tailchaing occured\r \n");
		high_intr = 0;
	}
	/* Turn off the LED. */
	GPIO_PORTF_DATA_R &= LED_OFF ;
	
	low_intr = 0;
	
}

void GPIOE_Handler(void){	
	int i;
	/* Clear interrupt flag for PE1 */
	GPIO_PORTE_ICR_R |= 0x02;
	/* Turn on the LED */
	GPIO_PORTF_DATA_R |= 0x04 ;
	/* Delay for a bit. */
	for ( i = 0 ; i < DELAY ; i++) ;
	if ((low_intr == 1) && (NVIC_PEND0_R == 0x02))
	{
		UARTTxString("Preemption occured\r \n");
		high_intr = 0;
	}
	else if (NVIC_PEND0_R == 0x02)
		high_intr = 1;
	
	/* Turn off the LED. */
	GPIO_PORTF_DATA_R &= (~0x04) ;
}
