#include "TM4C123.h"
#include "interrupts.h"
#include "uart0.h"

//System Initialization for Floating Point Unit
void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}

/*----------------UART0 initialization ----------------*/
void UARTInit(void)
{
	//Enable clock for UART0_ and GPIO PortA
	SYSCTL_RCGC_UART_R |= 0x01;					
	SYSCTL_RCGC_GPIO_R |= 0x01;			

	/*-------Configuring GPIO as UART -------------------*/
	//Enable digital I/O onPA0 and PA1
	GPIO_PORTA_DEN_R |= 0x03;					
	//Enable alternate functionality on PA0 and PA1
	GPIO_PORT_A_AFSEL_R |= 0x03;
  //Select uart0 on PA0 and PA1	
	GPIO_PORTA_PCTL_R |= 0x00000011;

	/*------------Configuration of UART0_module-----------*/
	//Disable UART
	UART0_CTL_R &= (~0x01);							
	//IBRD=int(16,000,000/(16*115,200))=int(8.6805)
	UART0_IBRD_R=8;
	//FBRD=int(0.6805*64+0.5)=44
	UART0_FBRD_R=44;
	//8bit word length,no parity bit,one stop bit,FIFOs enable
	UART0_LCRH_R = 0x70;
	//Use system clock as UART clock
	UART0_CC_R = 0x0;		
  //Enable loop back mode	
	//UART0_CTL_R|=0x80;
  //Enable UART0	
	UART0_CTL_R |= 0x01;							
}

/*---------------Interrupts initialization on port A and port C ----------*/
void InterruptInit(void)
{
	volatile unsigned delay_clk;
	
	/* Enable clock for PortB and PortE */
	SYSCTL_RCGC_GPIO_R |= 0x12;

	/* Dummy read to stable the clock */	
	delay_clk = SYSCTL_RCGC_GPIO_R;		
	
	/* Enable digital I/O on PE1 PB2, 
	* Make PE1, PB2 input
	* Enable weak pullup on PE1,PB2 */
	GPIO_PORTB_DEN_R |= 0x04;
	GPIO_PORTE_DEN_R |= 0x02;
	
	GPIO_PORTB_DIR_R &= (~0x04);
	GPIO_PORTE_DIR_R &= (~0x02);
	
	GPIO_PORTB_PUR_R |= 0x04;
	GPIO_PORTE_PUR_R |= 0x02;
	
	/* INTERRUPT Configuration */
	/* Globally disable all interrupts */
	DisableInterrupts();
	
	/* Disable all interrupts on PortB,E */
	GPIO_PORTB_IM_R &= 0x00;
	GPIO_PORTE_IM_R &= 0x00;
	
	/* Enable PortB (1),E (4) interrupt in NVIC */
	NVIC_EN0_R |= 0x00000012;
	
	/* Set priority of PortB as 4 and PortE as 2. */
	NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFF00FF) | 0x00008000;
	NVIC_PRI1_R = (NVIC_PRI1_R & 0xFFFFFF00) | 0x00000040;
	
	/* Configure PE1,PB2 as edge sensitive,
	* not both edges,
	* falling edge */
	GPIO_PORTB_IS_R &= (~0x04);
	GPIO_PORTE_IS_R &= (~0x02);
	
	GPIO_PORTB_IBE_R &= (~0x04);
	GPIO_PORTE_IBE_R &= (~0x02);
	
	GPIO_PORTB_IEV_R &= (~0x04);
	GPIO_PORTE_IEV_R &= (~0x02);
	
	/* Clear interrupt flag for PE1,PB2 */
	GPIO_PORTB_ICR_R |= 0x04 ;
  GPIO_PORTE_ICR_R |= 0x02;	
	
	/* Enable interrupt on PE1,PB2 */
	GPIO_PORTB_IM_R |= 0x04;	
	GPIO_PORTE_IM_R |= 0x02;

	/* Enable interrupts beyond the defined priority level */
	EnablePriorityInterrupts();
	/* Globally enable all interrupts */
	EnableInterrupts();
	
}

/*------------ Sending and receiving on uart ------------*/
//Wait for input and returns its ASCII value
unsigned char UARTRx(void)
{
	while((UART0_FR_R & UART_FR_RX_FE)!=0);
	return((unsigned char)(UART0_DR_R & 0xFF));
}

/*Accepts ASCII characters from the serial port and
adds them to a string.It echoes each character as it
is inputted.*/
void UARTRxString (char *pt, unsigned short max)
{
	int length=0;
	char character;

	character = UARTRx();
	if(length<max)
	{
		*pt=character;
		pt++;
		length++;
		UARTTx(character);
	}

	*pt=0;
}

//Output 8bit to serial port
void UARTTx(unsigned char data)
{
	while((UART0_FR_R & UART_FR_TX_FF)!=0);
	UART0_DR_R=data;
}


//Output a character string to serial port
void UARTTxString(char *pt)
{
	while(*pt)
	{
		UARTTx(*pt);
		pt++;
	}
}
