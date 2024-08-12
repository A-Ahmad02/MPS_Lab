/******************************************************************************
*	Project name: Lab 4
*	File name:
*	Author:
*	Date: 10-10-23
*	Description: GPIO interfacing for Blue LED (PF2)
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
#define SYSCTL_RCGCGPIO_R (*((volatile unsigned long *) 0x400FE608))
// Base Address of Port F (APB) is 0x4002_5000
#define GPIO_PORTF_DATA_R (*((volatile unsigned long*) 0x40025010))		// 0x010 -> 00_0001_0000
#define GPIO_PORTF_DIR_R (*((volatile unsigned long*) 0x40025400))		// 0x400 offset for GPIODIR
#define GPIO_PORTF_DEN_R (*((volatile unsigned long*) 0x4002551C))		// 0x51C offset for GPIODEN

/* Enable clock for PortF */
#define GPIO_PORTF_CLK_EN 0x20				// Binary (6 ports) 10_0000
/* PF3 GPIO configuration */
#define GPIO_PORTF_PIN2_EN 0x04				// Binary (5 pins) 0_0100
/* Green LED on */
#define LED_ON 0x04							// Set PF2 DATA =  1 to turn on LED
/* Green LED off */
#define LED_OFF ~(0x04)						// Set PF2 DATA =  0 to turn off LED
/* Delay */
#define DELAY 200000

int main ( void )
{
	volatile unsigned long ulLoop ;

	 /* Enable the GPIO port that is used for the on-board LED. */
	SYSCTL_RCGCGPIO_R |=  GPIO_PORTF_CLK_EN;
	 /* Do a dummy read to insert a few cycles after enabling the peripheral. */
	ulLoop = SYSCTL_RCGCGPIO_R ;
	
	/* Enable the GPIO pin for the LED (PF2) for digital function.
	Set the direction as output.*/
	GPIO_PORTF_DEN_R |=  GPIO_PORTF_PIN2_EN;	// Set PF2 as digital (=1)
	GPIO_PORTF_DIR_R |=  GPIO_PORTF_PIN2_EN;	// Set PF2 as output (=1)
	
	/* Loop forever */
	while (1)
	{
		 /* Turn on the LED */
		GPIO_PORTF_DATA_R |= LED_ON;
		/* Delay for a bit. */
		for ( ulLoop = 0 ; ulLoop < DELAY ; ulLoop++) ;
		
		/* Turn off the LED. */
		GPIO_PORTF_DATA_R &=  LED_OFF;
		/* Delay for a bit. */
		for ( ulLoop = 0 ; ulLoop < DELAY ; ulLoop++ ) ;
	}
}
