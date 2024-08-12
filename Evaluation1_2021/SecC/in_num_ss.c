#include "gpio_addr.h"

const char lut_display[16]={0xC0,
							0xF9,
							0xA4,
							0xB0,
							0x99,
							0x92,
							0x82,
							0xF8,
							0x80,
							0x90,
							0x88,
							0x83,
							0xC6,
							0xA1,
							0x86,
							0x8E
							};

void GPIOF_Handler(void){	
	int in_num;
	/* Clear interrupt flag for PF0 */
	GPIO_PORTF_ICR_R |= PORTF_PIN0;
	
	//take number from input
	in_num = (GPIO_PORTD_DATA_R & 0x0F);
	//display number on seven segment display
	GPIO_PORTA_DATA_R = 0x3C;
	GPIO_PORTB_DATA_R = lut_display[in_num];
	GPIO_PORTA_DATA_R = ~PORTA_PINS;
}

int main(){
	GPIOinit();
	Interruptinit();
	while(1)
	{
		WaitForInterrupt();
	}
}
