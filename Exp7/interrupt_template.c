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
#define	SYSCTL_RCGCGPIO_R		(*((volatile unsigned long*)0x400FE608))

/* Enable Register for PortF Interrupt */
// Base Address of NVIC (or SysTick, MPU, FPU and SCB) is 0xE000_E000 (from Table 2.4)
#define NVIC_EN0_R (*((volatile unsigned long*) 0xE000E100))		// 0x100 offset for Interrupt number 30 (0-31)

/* Priority Register for PortF Interrupt */
// Base Address of NVIC (or SysTick, MPU, FPU and SCB) is 0xE000_E000 (from Table 2.4)
#define NVIC_PRI7_R (*((volatile unsigned long*) 0xE000E41C))		// 0x41C offset for Interrupt number 30 (28-31)

/* Port F GPIO Registers */
// Base Address of Port F (APB) is 0x4002_5000
#define GPIO_PORTF_DATA_R (*((volatile unsigned long*) 0x40025078))		// 0x038 -> 00_0011_10_00
#define GPIO_PORTF_DIR_R (*((volatile unsigned long*) 0x40025400))		// 0x400 offset for GPIODIR
#define GPIO_PORTF_DEN_R (*((volatile unsigned long*) 0x4002551C))		// 0x51C offset for GPIODEN

#define GPIO_PORTF_IS_R (*((volatile unsigned long*) 0x40025404))			//  0x404 offset for GPIOIS
#define GPIO_PORTF_IBE_R (*((volatile unsigned long*) 0x40025408))		//  0x408 offset for GPIOIBE
#define GPIO_PORTF_IEV_R (*((volatile unsigned long*) 0x4002540C))		//  0x40C offset for GPIOIEV
#define GPIO_PORTF_IM_R (*((volatile unsigned long*) 0x40025410))			//  0x410 offset for GPIOIM
#define GPIO_PORTF_ICR_R (*((volatile unsigned long*) 0x4002541C))			//  0x41C offset for GPIOICR

#define GPIO_PORTF_PUR_R (*((volatile unsigned long*) 0x40025510))			//  0x510 offset for GPIOPUR (pull-up register)

/* Values to set */
#define NVIC_EN0_INT30 0x40000000 // 0100_0000_0000_0000_0000_0000_0000_0000 Interrupt 30 enable
#define GPIO_PORTF_CLK_EN 0x20		// (6 ports) 10_0000 to Clock enable for PortF
#define GPIO_PORTF_LEDS_EN 0x0E		// (5 pins) 0_1110 to Enable LEDs (PF1, PF2 and PF3)
#define GPIO_PORTF_SW1_EN 0x10		// (5 pins) 1_0000 to Enable user switch SW1 (PF4)
#define INT_PF4 0x10 							// (5 pins) 1_0000 as Interrupt at PF4

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

// To enable floating point for __main that needs it
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
	SYSCTL_RCGCGPIO_R |=  GPIO_PORTF_CLK_EN;
	
	/* Dummy read to stable the clock */	
	delay_clk = SYSCTL_RCGCGPIO_R;		
	
	
	/* Enable digital I/O on PF4,PF3-PF1, 
	* Make PF4 input and PF3-PF1 output
	* Enable weak pullup on PF4 */
	GPIO_PORTF_DEN_R |= (GPIO_PORTF_LEDS_EN|GPIO_PORTF_SW1_EN); 
	
	GPIO_PORTF_DIR_R |= GPIO_PORTF_LEDS_EN;		// Set PF1,2,3 as output (=1)
	GPIO_PORTF_DIR_R &= ~GPIO_PORTF_SW1_EN; 		// Set PF4 as input (=0)
	
	GPIO_PORTF_PUR_R |= GPIO_PORTF_SW1_EN;		// Enable weak pullup for PF4
	
	/* INTERRUPT Configuration */
	/* Globally disable all interrupts */
	DisableInterrupts();
	
	/* Enable PortF interrupt in NVIC */
	NVIC_EN0_R |= NVIC_EN0_INT30;
	
	/* Set priority of PortF as 5 */
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000 ; // A0 -> 1010_0000 as 101 is 5
	
	/* Disable all interrupts on PortF */
	GPIO_PORTF_IM_R &= 0x00;
	
	/* Configure PF4 as edge sensitive,
	* not both edges,
	* falling edge */
	GPIO_PORTF_IS_R &= (~INT_PF4); 	// Set as 0 for PF4 as edge sensitive
	GPIO_PORTF_IBE_R &= (~INT_PF4); // Set as 0 for PF4 as single edge sensitive
	GPIO_PORTF_IEV_R &= (~INT_PF4); // Set as 0 for PF4 as falling edge sensitive
	
	/* Clear interrupt flag for PF4 */
	GPIO_PORTF_ICR_R |= INT_PF4;		// Set as 1 to clear	
	
	/* Enable interrupt on PF4 */
	GPIO_PORTF_IM_R |= INT_PF4;	

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
		//GPIO_PORTF_DATA_R |= 0x4;
		WaitForInterrupt();
	}
}
