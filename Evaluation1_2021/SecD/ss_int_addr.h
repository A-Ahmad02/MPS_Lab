#include "TM4C123.h"

/* Register for clock */
#define	SYSCTL_RCGC_GPIO_R	(*((volatile unsigned long*)0x400FE608))

/* GPIO Registers for port A */	
#define	GPIO_PORTA_DATA_R		(*((volatile unsigned long*)0x400043FC))
#define	GPIO_PORTA_DIR_R		(*((volatile unsigned long*)0x40004400))
#define	GPIO_PORTA_DEN_R		(*((volatile unsigned long*)0x4000451C))

/* GPIO Registers for port B */	
#define	GPIO_PORTB_DATA_R		(*((volatile unsigned long*)0x400053FC))
#define	GPIO_PORTB_DIR_R		(*((volatile unsigned long*)0x40005400))
#define	GPIO_PORTB_DEN_R		(*((volatile unsigned long*)0x4000551C))

/* GPIO Registers for port D */	
#define	GPIO_PORTD_DATA_R		(*((volatile unsigned long*)0x400073FC))
#define	GPIO_PORTD_DIR_R		(*((volatile unsigned long*)0x40007400))
#define	GPIO_PORTD_DEN_R		(*((volatile unsigned long*)0x4000751C))

/* GPIO Registers for port F */	
#define GPIO_PORTF_DATA_R		(*((volatile unsigned long*)0x400253FC))
#define GPIO_PORTF_DIR_R		(*((volatile unsigned long*)0x40025400))
#define GPIO_PORTF_DEN_R		(*((volatile unsigned long*)0x4002551C))
#define	GPIO_PORTF_PUR_R		(*((volatile unsigned long*)0x40025510))
#define	GPIO_PORTF_LOCK_R		(*((volatile unsigned long*)0x40025520))
#define GPIO_PORTF_CR_R		  (*((volatile unsigned long*)0x40025524))

/*Interrupt Registers for portF */
#define GPIO_PORTF_IS_R			(*((volatile unsigned long*)0x40025404))
#define GPIO_PORTF_IBE_R		(*((volatile unsigned long*)0x40025408))
#define GPIO_PORTF_IEV_R		(*((volatile unsigned long*)0x4002540C))
#define GPIO_PORTF_IM_R			(*((volatile unsigned long*)0x40025410))
#define GPIO_PORTF_ICR_R		(*((volatile unsigned long*)0x4002541C))
#define GPIO_PORTF_MIS_R		(*((volatile unsigned long*)0x40025418))

#define NVIC_EN0_R					(*((volatile unsigned long*)0xE000E100))
#define NVIC_PRI7_R					(*((volatile unsigned long*)0xE000E41C))


/* PortA pins 2-5, portB pins 0-7 PORTD pins 4-7 */
#define PORTA_PINS 0x3C
#define PORTB_PINS 0xFF
#define PORTD_PINS 0x03

#define PORTF_PINS 0x11
#define INT_PF0    0x01
#define INT_PF4    0x10

//Function Declarations
void SSinit(void);
void Interruptinit(void);
void delay(unsigned long value);
void EnableInterrupts(void);
void DisableInterrupts(void);	
void EnablePriorityInterrupts(void);
void WaitForInterrupt(void);
void GPIOF_Handler(void);
