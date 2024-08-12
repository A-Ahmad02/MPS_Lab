/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: This is a template file for GPIO parallel interfacing for
*				Seven Segment Display. This program is written for common anode
*				type seven segment display
* 				Seven segment digits pins:		PA2-PA5*
*				Seven segment data pins:				PB0-PB7*
*				Port B pins:							76543210*
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
#define	SYSCTL_RCGCGPIO_R		(*((volatile unsigned long*)0x400FE608))

/* Enable clock for Port A and B */
#define GPIO_PORTAB_CLK_EN 0x03				// Binary (6 ports) 00_0011

/* PA2-5 GPIO configuration */
#define GPIO_PORTA_PIN2_5_EN 0x3C				// Binary (8 pins) 0011_1100

/* PB0-7 GPIO configuration */
#define GPIO_PORTB_PIN0_7_EN 0xFF				// Binary (8 pins) 1111_1111

/* GPIO Registers for port B */		// Base Address of Port B (APB) is 0x40005000
#define	GPIO_PORTB_DATA_R		(*((volatile unsigned long*)0x400053FC))	// 0x3FC -> 11_1111_1100
#define	GPIO_PORTB_DIR_R		(*((volatile unsigned long*)0x40005400))
#define	GPIO_PORTB_DEN_R		(*((volatile unsigned long*)0x4000551C))

/* GPIO Registers for port A */	// Base Address of Port A (APB) is 0x40004000
#define	GPIO_PORTA_DATA_R		(*((volatile unsigned long*)0x400040F0))	// 0x0F0 -> 00_1111_0000
#define	GPIO_PORTA_DIR_R		(*((volatile unsigned long*)0x40004400))
#define	GPIO_PORTA_DEN_R		(*((volatile unsigned long*)0x4000451C))

/* Values for enabling seven segments */
#define	SEG_1			0xFB			// PA2 -> 1111_1011
#define	SEG_2			0xF7			// PA3 -> 1111_0111 
#define	SEG_3			0xEF			// PA4 -> 1110_1111  
#define	SEG_4			0xDF			// PA5 -> 1101_1111 
#define	SEG_OFF		0xFF			// 1111_1111

#define SEG_DELAY 10000
#define REPEAT 100

/* Function Declarations */	
void init_gpio(void);
void display_1(void);
void display_2(void);
void delay(unsigned long value);

/*Lookup tables for common anode display */

/*lut for display1*/
const char lut_display1[4]={0xA4,//2 --> 1010_0100
														0xC0,//0 --> 1100_0000
														0xA4,//2 --> 1010_0100 
														0xF9 //1 -->1111_1001  
							};
							
/* lut for display2 */
const char lut_display2[4]={0x89,//H --> 1000_1001
														0x86,//E --> 1000_0110
														0xC7,//L --> 1100_0111
														0xC0 //O --> 1100_0000
							};

/* lut for segment selection */
const char seg_select[]={SEG_4,//SEG_1
												 SEG_3,//SEG_2
												 SEG_2,//SEG_3
												 SEG_1 //SEG_4
						};
						
/*initialization function for ports */
void init_gpio(void){
	volatile unsigned long delay_clk;
	
	/*enable clock for PortA and PortB */
	SYSCTL_RCGCGPIO_R |=  GPIO_PORTAB_CLK_EN;
	
	// dummy read for delay for clock,must have 3sys clock delay
	delay_clk=SYSCTL_RCGCGPIO_R;
	
	/* Enable the GPIO pin for PortB pins 0-7 for digital function
	and set the direction as output. */	
	GPIO_PORTB_DEN_R |=  GPIO_PORTB_PIN0_7_EN;
	GPIO_PORTB_DIR_R |=  GPIO_PORTB_PIN0_7_EN;
	
	/* Enable the GPIO pin for PortA pins 2-5 for digital function.
	and set the direction as output. */
	GPIO_PORTA_DEN_R |=  GPIO_PORTA_PIN2_5_EN;
	GPIO_PORTA_DIR_R |=  GPIO_PORTA_PIN2_5_EN;
	
}

/* display_1 on seven segments using Macros */
void display_1(void){
	int i,j;
	for(j=0;j<REPEAT;j++){
		for(i=0;i<4;i++)
		{
			GPIO_PORTA_DATA_R=SEG_OFF;
			GPIO_PORTB_DATA_R=lut_display1[i];
			GPIO_PORTA_DATA_R=seg_select[i];
			delay(SEG_DELAY);
		}
	}
}
/*
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
*/
/* display_2 on seven segments using for loop */
void display_2(void){
	int i,j;
	for(j=0;j<REPEAT;j++)
		for(i=0;i<4;i++)
		{
			GPIO_PORTA_DATA_R=SEG_OFF;
			GPIO_PORTB_DATA_R=lut_display2[i];
			GPIO_PORTA_DATA_R=seg_select[i];
			delay(SEG_DELAY);
		}
	}

/* Delay function */
void delay(unsigned long value){
	unsigned long i ;
	for(i=0;i<2*value;i++);
}

/* Main function */
int main(void){
	init_gpio();
	while(1){
		display_1();
		display_2();
	}
}
