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
// Base address of PortD (APB) 0x4000_4000
#define GPIO_PORTA_AFSEL_R			(*((volatile unsigned long*)0x40004420))	// 0x420 offset for AFSEL
#define GPIO_PORTA_PCTL_R			(*((volatile unsigned long*)0x4000452C))	// 0x52C offset for PTCL
#define GPIO_PORTA_DEN_R			(*((volatile unsigned long*)0x4000451C))	// 0x51C offset for DEN
#define GPIO_PORTA_DIR_R			(*((volatile unsigned long*)0x40004400))	// 0x400 offset for DIR

//Register definitions for UART2_ module
// Base address of UART2 is 0x4000_C000
#define 	UART0_CTL_R						(*((volatile unsigned long*)0x4000C030))	// 0x030 offset for UARTCTL
#define  	UART0_IBRD_R					(*((volatile unsigned long*)0x4000C024))	// 0x024 offset for UARTIBRD
#define  	UART0_FBRD_R					(*((volatile unsigned long*)0x4000C028))	// 0x028 offset for UARTFBRD
#define 	UART0_LCRH_R					(*((volatile unsigned long*)0x4000C02C))	// 0x02C offset for UARTLCRH
#define 	UART0_CC_R						(*((volatile unsigned long*)0x4000CFC8))	// 0xFC8 offset for UARTCC
#define 	UART0_FR_R						(*((volatile unsigned long*)0x4000C018))	// 0x018 offset for UARTFR
#define 	UART0_DR_R						(*((volatile unsigned long*)0x4000C000))	// 0x000 offset for UARTDR

//Macros
#define 	UART_FR_TX_FF				0x20	// 0010_0000 bit 5 is 1 when UART Transmit FIFO Full
#define 	UART_FR_RX_FE				0x10	// 0001_0000 bit 4 is 1 when UART Receive FIFO Empty

//Function definitions
unsigned char UARTRx(void);
void UARTTx(unsigned char data);
void UARTTxString(char*pt);
void UARTRxString(void);

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
	//Enable clock for UART0_ and respective GPIO Port
	SYSCTL_RCGC_GPIO_R |= 0x01; // 00_0001 to enable port A
	delay_clk = SYSCTL_RCGC_GPIO_R;
	SYSCTL_RCGC_UART_R |= 0x01; // 0000_0001 to enable UART 0 (UART 0-7)
	delay_clk = SYSCTL_RCGC_UART_R;
	
	// Configure GPIO as UART (AFSEL,PCTL,DEN)
	GPIO_PORTA_DEN_R |= 0x03; 				// 0000_0011 enable pins PA1(Rx) and PA0(Tx)
	GPIO_PORTA_AFSEL_R |= 0x03; 			// 0000_0011 Set(=1) enable alternate functionality PD6(Rx) and PD7(Tx)
	GPIO_PORTA_PCTL_R |= 0x00000011; 	// 0000_0000_0000_0000_0000_0000_0001_0001 port mux control for pins PD6 and PD7 (from Table 23-5)
	
	//Disable UART
	UART0_CTL_R &= ~(0x1); // 0001 Disable Uart by setting UARTEN=0 (bit 0 of UARTCTL)
	
	//Select system clock/PIOSC as UART Baud clock
	UART0_CC_R = 0x0; // 0000 (lower 4 bits) selct system as UART clock
										// 0101 select PIOSC as UART clock
	
	//Set Baud Rate
	// CLKDIV = 8(HSE=1) or 16(HSE=0) (HSE is bit5 in UARTCTL)
	// BRD = BRDI+BRDF = UARTSysClk/(ClkDiv*BaudRate)
	// BRD (16 bit) integer part of BRD
	// FBRD (6 bit) fractional part of BRD 
	// BRD = 16M/(16*115,200) = 8.6805
	UART0_CTL_R &= ~(0x20); // 0010_0000 HSE=0 for CLKDIV=16
	UART0_IBRD_R = 0x0008; 	// _0001 (lower 16 bits) IBRD=8 (in DIVINT field of UARTIBRD)
	UART0_FBRD_R = 0x2C; 		// 10_1100 (lower 6 bits) FBRD=integer(0.6805*64+0.5)=44 (in DIVFRAC field of UARTFBRD)
	
	//8bit word length,no parity bit,one stop bit,FIFOs enable
	UART0_LCRH_R &= ~(0x2); 		// 0010 Parity enable=0 (is bit1 in UARTLCRH)
	UART0_LCRH_R &= ~(0x8);  		// 1000 TwoStopBits=0 (is bit3 in UARTLCRH)
	UART0_LCRH_R |= 0x10;  	// 0001_0000 EnableFIFOs=1 (is bit4 in UARTLCRH)
	UART0_LCRH_R |= 0x60; 	// 0110_0000  WordLength=8 (bit5and6 in UARTLCRH)
	
	//Enable UART
	UART0_CTL_R |= 0x1; // 0001 Enable Uart by setting UARTEN=1 (bit 0 of UARTCTL)
	
}

//Wait for input and returns its ASCII value
unsigned char UARTRx(void)
{
	while((UART0_FR_R & UART_FR_RX_FE)!=0); // Check if Receive FIFO Empty stay in loop
	return((unsigned char)(UART0_DR_R & 0xFF));
}

/*Accepts ASCII characters from the serial port and
adds them to a string.It echoes each character as it
is inputted.*/
void UARTRxString (void)
{
	char char_;
	char_ = UARTRx();
	UARTTx(char_);

}

//Output 8bit to serial port
void UARTTx(unsigned char data)
{
	while((UART0_FR_R & UART_FR_TX_FF)!=0); // Check if Transmit FIFO Full stay in loop
	UART0_DR_R=data;
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
	UARTInit();
	//The input given using keyboard is displayed on hyper terminal
	//.i.e.,data is echoed
	UARTTxString("EnterText:");

while(1)
	{
		UARTRxString();
	}
}
