/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: This is a template file for GPIO interfacing for Green LED
*
 ******************************************************************************/

#include "TM4C123.h"

void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}
	

/******************************************************************************
*Macros for Register Addresses and Values
*******************************************************************************/	
#define SYSCTL_RCGCGPIO_R (*((volatile unsigned long*) 0x))
#define GPIO_PORTF_DATA_R (*((volatile unsigned long*) 0x))
#define GPIO_PORTF_DIR_R (*((volatile unsigned long*) 0x))
#define GPIO_PORTF_DEN_R (*((volatile unsigned long*) 0x))

/* Enable clock for PortF */
#define GPIO_PORTF_CLK_EN 0x
/* PF3 GPIO configuration */
#define GPIO_PORTF_PIN3_EN 0x
/* Green LED on */
#define LED_ON 0x
/* Green LED off */
#define LED_OFF ~(0x)
/* Delay */
#define DELAY 200000

int main ( void )
{
	volatile unsigned long ulLoop ;

	 /* Enable the GPIO port that is used for the on-board LED. */
	SYSCTL_RCGCGPIO_R |=  ;
	 /* Do a dummy read to insert a few cycles after enabling the peripheral. */
	ulLoop = SYSCTL_RCGCGPIO_R ;
	
	/* Enable the GPIO pin for the LED (PF3) for digital function.
	Set the direction as output.*/
	GPIO_PORTF_DEN_R |=  ;
	GPIO_PORTF_DIR_R |=  ;
	
	/* Loop forever */
	while (1)
	{
		 /* Turn on the LED */
		GPIO_PORTF_DATA_R |= ;
		/* Delay for a bit. */
		for ( ulLoop = 0 ; ulLoop < DELAY ; ulLoop++) ;
		/* Turn off the LED. */
		GPIO_PORTF_DATA_R &=  ;
		/* Delay for a bit. */
		for ( ulLoop = 0 ; ulLoop < DELAY ; ulLoop++ ) ;
	}
}
