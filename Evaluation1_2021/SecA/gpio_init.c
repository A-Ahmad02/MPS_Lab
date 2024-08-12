#include "uart0.h"
#include "gpio.h"

void GPIOinit ( void )
{
	volatile unsigned long ulLoop ;
	
	/* Enable the GPIO port that is used for the on-board LED. */
	SYSCTL_RCGC_GPIO_R |= GPIO_PORTF_CLK_EN ;
	/* Do a dummy read to insert a few cycles after enabling the peripheral. */
	ulLoop = SYSCTL_RCGC_GPIO_R ;
	/* Enable the GPIO pin for the LED (PF3) for digital function.*/
	GPIO_PORTF_DEN_R |= GPIO_PORTF_PIN3_EN ;
	/* Set the direction as output.*/
	GPIO_PORTF_DIR_R |= GPIO_PORTF_PIN3_EN ;
}
