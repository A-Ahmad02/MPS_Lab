/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: This is a template file for GPIO parallel interfacing for
*				Seven Segment Display. This program is written for common anode
*				type seven segment display
* 				Seven segment digits pins:		PA2-PA5*
*				Seven segment data pins:		PB0-PB7*
*				Port B pins:					76543210*
*				Seven Segment LEDs:				pgfedcba*
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
/* Register for clock */
#define	SYSCTL_RCGCGPIO_R		(*((volatile unsigned long*)0x))

/* GPIO Registers for port B */	
#define	GPIO_PORTB_DATA_R		(*((volatile unsigned long*)0x))
#define	GPIO_PORTB_DIR_R		(*((volatile unsigned long*)0x))
#define	GPIO_PORTB_DEN_R		(*((volatile unsigned long*)0x))

/* GPIO Registers for port A */	
#define	GPIO_PORTA_DATA_R		(*((volatile unsigned long*)0x))
#define	GPIO_PORTA_DIR_R		(*((volatile unsigned long*)0x))
#define	GPIO_PORTA_DEN_R		(*((volatile unsigned long*)0x))

/* Values for enabling seven segments */
#define	SEG_1		0x
#define	SEG_2		0x
#define	SEG_3		0x
#define	SEG_4		0x
#define	SEG_OFF		0xFF

/* Function Declarations */	
void init_gpio(void);
void display_1(void);
void display_2(void);
void delay(unsigned long value);

/*Lookup tables for common anode display */

/*lut for display1*/
const char lut_display1[4]={0x,//U --> 11000001
							0x,//O --> 11000000
							0x,//F --> 10001110
							0x//S --> 10010010 
							};
							
/* lut for display2 */
const char lut_display2[4]={0x,//0x89,H --> 10001001
							0x,//E --> 10000110
							0x,//L --> 11000111
							0x//O --> 11000000
							};

/* lut for segment selection */
const char seg_select[]={0x,//SEG_1
						 0x,//SEG_2
						 0x,//SEG_3
						 0x//SEG_4
						};
						
/*initialization function for ports */
void init_gpio(void){
	volatile unsigned long delay_clk;
	
	/*enable clock for PortA and PortB */
	
	
	// dummy read for delay for clock,must have 3sys clock delay
	delay_clk=SYSCTL_RCGCGPIO_R;
	
	/* Enable the GPIO pin for PortB pins 0-7 for digital function
	and set the direction as output. */	
	
	
	
	/* Enable the GPIO pin for PortA pins 2-5 for digital function.
	and set the direction as output. */
	
	
	
}

/* display_1 on seven segments using Macros */
void display_1(void){
	GPIO_PORTA_DATA_R =SEG_OFF;
	GPIO_PORTB_DATA_R =lut_display1[0];
	GPIO_PORTA_DATA_R =SEG_1;
	delay(10000);
	GPIO_PORTA_DATA_R=SEG_OFF;
	GPIO_PORTB_DATA_R=lut_display1[1];
	GPIO_PORTA_DATA_R=SEG_2;
	delay(10000);
	GPIO_PORTA_DATA_R=SEG_OFF;
	GPIO_PORTB_DATA_R=lut_display1[2];
	GPIO_PORTA_DATA_R=SEG_3;
	delay(10000);
	GPIO_PORTA_DATA_R=SEG_OFF;
	GPIO_PORTB_DATA_R=lut_display1[3];
	GPIO_PORTA_DATA_R=SEG_4;
	delay(10000);
}

/* display_2 on seven segments using for loop */
void display_2(void){
	int i;
	for(i=0;i<4;i++)
	{
		GPIO_PORTA_DATA_R=SEG_OFF;
		GPIO_PORTB_DATA_R=lut_display2[i];
		GPIO_PORTA_DATA_R=seg_select[i];
		delay(10000);
	}
}

/* Delay function */
void delay(unsigned long value){
	unsigned long i ;
	for(i=0;i<value;i++);
}

/* Main function */
int main(void){
	int i;
	init_gpio();
	while(1)
		display_1();
		//display_2();
	
}
