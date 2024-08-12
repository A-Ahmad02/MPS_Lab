/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: This is a template file for Interrupt configuration for 
*                User button connected to PF4. It turns on different LEDs
*                on falling edge of button press
 ******************************************************************************/


#include "TM4C123.h"
/******************************************************************************
*Macros for Register Addresses and Values
*******************************************************************************/

/* Register for GPIO clock */

/* Enable Register for PortF Interrupt */

/* Priority Register for PortF Interrupt */

/* Port F GPIO Registers */


		

/* Function Declarations */	
/*-----These are functions added in startup file-----*/
/* Enable interrupts */
void EnableInterrupts(void);
/* Disableinterrupts */									
void DisableInterrupts(void);
/* Enable Priority interrupts */
void EnablePriorityInterrupts(void);
/* Implements WFI*/							
void WaitForInterrupt(void);

/*----These are C frunctions of this file-------*/
/* Initialize GPIO and Interrupts */
void Init_INT_GPIO(void);
/* Implements delay	*/
void Delay(unsigned long value);
/* Global Variable */
volatile unsigned long i=0;


void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}


void Init_INT_GPIO(void){
	volatile unsigned delay_clk;
	
	/* Enable clock for PORTF */

	/* Dummy read to stable the clock */	
	delay_clk = SYSCTL_RCGCGPIO_R;		
	
	
	/* Enable digital I/O on PF4,PF3-PF1, 
	* Make PF4 input and PF3-PF1 output
	* Enable weak pullup on PF4 */
	
	
	
	
	/* INTERRUPT Configuration */
	/* Globally disable all interrupts */
	DisableInterrupts();
	
	/* Enable PortF interrupt in NVIC */
	
	
	/* Set priority of PortF as 5 */
	
	
	/* Disable all interrupts on PortF */
	GPIO_PORTF_IM_R &= 0x00;
	
	/* Configure PF4 as edge sensitive,
	* not both edges,
	* falling edge */
	
	
	
	
	/* Clear interrupt flag for PF4 */
	GPIO_PORTF_ICR_R ;	
	
	/* Enable interrupt on PF4 */
	GPIO_PORTF_IM_R |= ;	

	/* Enable interrupts beyond the defined priority level */
	EnablePriorityInterrupts();
	/* Globally enable all interrupts */
	EnableInterrupts();
}


void Delay(unsigned long value){
	unsigned long i=0;
	for(i=0;i<value;i++);
}

/* Interrupt Service Routine for PortF */
void GPIOF_Handler(void){
		int j;
		/* Clear interrupt flag for PF4 */
		GPIO_PORTF_ICR_R |= INT_PF4;
		
		/* Turn on different LEDs on each button press */
		if(i>=3)
		i=1;
		else
		i++;
		for(j=0;j<2;j++)
		{
			GPIO_PORTF_DATA_R^=1<<i;
			Delay(10000000);
		}
}

/* Main Function */
int main(){
	Init_INT_GPIO();
	while(1)
	{
		WaitForInterrupt();
	}
}
