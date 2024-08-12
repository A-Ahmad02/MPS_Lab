/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: This is a template file for UART0 configuration. It echoes back
*				 the character sent.
 ******************************************************************************/

#include "TM4C123.h"
/******************************************************************************
*Macros for Register Addresses and Values
*******************************************************************************/
//Register definitions for ClockEnable
// System Control base address of 0x400F_E000
#define 	SYSCTL_RCGC_UART_R				(*((volatile unsigned long*)0x400FE618))	// 0x618 offset for RCGCUART
#define 	SYSCTL_RCGC_GPIO_R				(*((volatile unsigned long*)0x400FE608))	// 0x608 offset for RCGCGPIO

//Register definitions for GPIOPortD
// Base address of PortD (APB) 0x4000_7000
#define 	GPIO_PORTD_AFSEL_R			(*((volatile unsigned long*)0x40007420))	// 0x420 offset for AFSEL
#define 	GPIO_PORTD_PCTL_R				(*((volatile unsigned long*)0x4000752C))	// 0x52C offset for PTCL
#define 	GPIO_PORTD_DEN_R				(*((volatile unsigned long*)0x4000751C))	// 0x51C offset for DEN
#define 	GPIO_PORTD_DIR_R				(*((volatile unsigned long*)0x40007400))	// 0x400 offset for DIR

#define GPIO_PORTD_LOCK_R					(*((volatile unsigned long*)0x40007520))	// 0x520 offset for LOCK
#define GPIO_PORTD_CR_R						(*((volatile unsigned long*)0x40007524))	// 0x524 offset for CR

//Register definitions for UART2_ module
// Base address of UART2 is 0x4000_E000
#define 	UART2_CTL_R						(*((volatile unsigned long*)0x4000E030))	// 0x030 offset for UARTCTL
#define  	UART2_IBRD_R					(*((volatile unsigned long*)0x4000E024))	// 0x024 offset for UARTIBRD
#define  	UART2_FBRD_R					(*((volatile unsigned long*)0x4000E028))	// 0x028 offset for UARTFBRD
#define 	UART2_LCRH_R					(*((volatile unsigned long*)0x4000E02C))	// 0x02C offset for UARTLCRH
#define 	UART2_CC_R						(*((volatile unsigned long*)0x4000EFC8))	// 0xFC8 offset for UARTCC
#define 	UART2_FR_R						(*((volatile unsigned long*)0x4000E018))	// 0x018 offset for UARTFR
#define 	UART2_DR_R						(*((volatile unsigned long*)0x4000E000))	// 0x000 offset for UARTDR

//Macros
#define 	UART_FR_TX_FF				0x20								//UART Transmit FIFO Full
#define 	UART_FR_RX_FE				0x10								//UART Receive FIFO Empty

//Function definitions
unsigned char UARTRx(void);
void UARTTx(unsigned char data);
void UARTTxString(char*pt);
void UARTRxString(char*bufPt, unsigned short max);

// To enable floating point for __main that needs it
void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}

//Intialize and configure UART
void UARTInit(void)
{
	volatile unsigned delay_clk;
	//Enable clock for UART2_ and respective GPIO Port
	SYSCTL_RCGC_GPIO_R |= 0x08; // 00_1000 to enable port D
	delay_clk = SYSCTL_RCGC_GPIO_R;
	SYSCTL_RCGC_UART_R |= 0x04; // 0000_0100 to enable UART 2 (UART 0-7)
	delay_clk = SYSCTL_RCGC_UART_R;
	
	// Unlocking CR to enable PD7 alternate functionality (U2Tx)
	GPIO_PORTD_LOCK_R = 0x4C4F434B;	// Unlock CR 
	GPIO_PORTD_CR_R |= 0xFF;				// 1111_1111 (lower 8 bits) Unlock write protection
	
	// Configure GPIO as UART (AFSEL,PCTL,DEN)
	GPIO_PORTD_DEN_R |= 0xC0; 				// 1100_0000 enable pins PD6(Rx) and PD7(Tx)
	GPIO_PORTD_AFSEL_R |= 0xC0; 			// 1100_0000 Set(=1) enable alternate functionality PD6(Rx) and PD7(Tx)
	GPIO_PORTD_PCTL_R |= 0x11000000; 	// 0001_0001_0000_0000_0000_0000_0000_0000 port mux control for pins PD6 and PD7 (from Table 23-5)
	
	//Disable UART
	UART2_CTL_R &= ~(0x1); // 0001 Disable Uart by setting UARTEN=0 (bit 0 of UARTCTL)
	
	//Select system clock/PIOSC as UART Baud clock
	UART2_CC_R = 0x0; // (lower 4 bits) selct systemclock
	
	//Set Baud Rate
	// CLKDIV = 8(HSE=1) or 16(HSE=0) (HSE is bit5 in UARTCTL)
	// BRD = BRDI+BRDF = UARTSysClk/(ClkDiv*BaudRate)
	// BRD (16 bit) integer part of BRD
	// FBRD (6 bit) fractional part of BRD 
	// BRD = 16M/(16*115,200) = 8.6805
	UART2_CTL_R &= ~(0x20); // 0010_0000 HSE=0 for CLKDIV=16
	UART2_IBRD_R = 0x0008; 	// _0001 (lower 16 bits) IBRD=8 (in DIVINT field of UARTIBRD)
	UART2_FBRD_R = 0x2C; 		// 10_1100 (lower 6 bits) FBRD=integer(0.6805*64+0.5)=44 (in DIVFRAC field of UARTFBRD)
	
	//8bit word length,no parity bit,one stop bit,FIFOs enable
	UART2_LCRH_R &= ~(0x2); 		// 0010 Parity enable=0 (is bit1 in UARTLCRH)
	UART2_LCRH_R &= ~(0x8);  		// 1000 TwoStopBits=0 (is bit3 in UARTLCRH)
	UART2_LCRH_R |= 0x10;  	// 0001_0000 EnableFIFOs=1 (is bit4 in UARTLCRH)
	UART2_LCRH_R |= 0x60; 	// 0110_0000  WordLength=8 (bit5and6 in UARTLCRH)
	
	//Enable UART
	UART2_CTL_R |= 0x1; // 0001 Enable Uart by setting UARTEN=1 (bit 0 of UARTCTL)
	
}

//Wait for input and returns its ASCII value
unsigned char UARTRx(void)
{
	while((UART2_FR_R & UART_FR_RX_FE)!=0);
	return((unsigned char)(UART2_DR_R & 0xFF));
}

/*Accepts ASCII characters from the serial port and
adds them to a string.It echoes each character as it
is inputted.*/
void UARTRxString (char *pt, unsigned short max)
{
	int len = 0;
	char char_;
	while(len < max){
		char_ = UARTRx();
		*pt = char_;
		pt++;
		len++;
		UARTTx(char_);
}
	*pt = '\0';
}

//Output 8bit to serial port
void UARTTx(unsigned char data)
{
	while((UART2_FR_R & UART_FR_TX_FF)!=0);
	UART2_DR_R=data;
}


//Output a character string to serial port
void UARTTxString(char *pt)
{
	while(*pt){
		UARTTx(*pt);
		pt++;
	}
}

int main (void)
{
	char string[17];
	UARTInit();
	//The input given using keyboard is displayed on hyper terminal
	//.i.e.,data is echoed
	UARTTxString("EnterText:");

while(1)
	{
		UARTRxString(string,16);
	}
}
