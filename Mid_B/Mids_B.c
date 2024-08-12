/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: This is a template file for GPIO interfacing for Green LED
*
 ******************************************************************************/

#include "TM4C123.h"
#include "address.h"

//Function definitions
void UART0Tx(unsigned char data);
unsigned char UART2Rx(void);

void UARTTxString(char*pt);
void UARTRxString(char*bufPt, unsigned short max);

//Macros
#define 	UART_FR_TX_FF				0x20								//UART Transmit FIFO Full
#define 	UART_FR_RX_FE				0x10								//UART Receive FIFO Empty


void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}
	

//Intialize and configure UART0
void UART0Init(void)
{
	volatile unsigned delay_clk;
	//Enable clock for UART0_ and respective GPIO Port
	SYSCTL_RCGC_GPIO_R |= 0x01; // 00_1000 to enable port A
	delay_clk = SYSCTL_RCGC_GPIO_R;
	SYSCTL_RCGC_UART_R |= 0x01; // 0000_0001 to enable UART 0 (UART 0-7)
	delay_clk = SYSCTL_RCGC_UART_R;
	
	// Configure GPIO as UART (AFSEL,PCTL,DEN)
	GPIO_PORTA_DEN_R |= 0x03; 				// 0000_0011 enable pins PA1(Rx) and PA0(Tx)
	GPIO_PORTA_AFSEL_R |= 0x03; 			// 0000_0011 Set(=1) enable alternate functionality PD6(Rx) and PD7(Tx)
	GPIO_PORTA_PCTL_R |= 0x00000011; 	// 0000_0000_0000_0000_0000_0000_0001_0001 port mux control for pins PD6 and PD7 (from Table 23-5)
	
	//Disable UART
	UART0_CTL_R &= ~(0x301); // 0001 Disable Uart by setting UARTEN=0 (bit 0 of UARTCTL)
	
	//Select system clock/PIOSC as UART Baud clock
	UART0_CC_R = 0x0; // 0000 (lower 4 bits) selct system as UART clock
										// 0101 select PIOSC as UART clock
	
	//Set Baud Rate
	// CLKDIV = 8(HSE=1) or 16(HSE=0) (HSE is bit5 in UARTCTL)
	// BRD = BRDI+BRDF = UARTSysClk/(ClkDiv*BaudRate)
	// BRD (16 bit) integer part of BRD
	// FBRD (6 bit) fractional part of BRD 
	// BRD = 16M/(16*9600) = 104.166
	UART0_CTL_R &= ~(0x20); // 0010_0000 HSE=0 for CLKDIV=16
	UART0_IBRD_R = 0x0068; 	// (lower 16 bits) IBRD=104 (in DIVINT field of UARTIBRD)
	UART0_FBRD_R = 0x0B; 		// (lower 6 bits) FBRD=integer(0.166666*64+0.5)=11 (in DIVFRAC field of UARTFBRD)
	
	//8bit word length,no parity bit,one stop bit,FIFOs enable
	UART0_LCRH_R &= ~(0x2); 		// 0010 Parity enable=0 (is bit1 in UARTLCRH)
	UART0_LCRH_R &= ~(0x8);  		// 1000 TwoStopBits=0 (is bit3 in UARTLCRH)
	UART0_LCRH_R |= 0x10;  	// 0001_0000 EnableFIFOs=1 (is bit4 in UARTLCRH)
	UART0_LCRH_R |= 0x40; 	// 0100_0000  WordLength=7 (bit5and6 in UARTLCRH)
	
	//Enable UART
	UART0_CTL_R |= 0x301; // 0001 Enable Uart by setting UARTEN=1 (bit 0 of UARTCTL)
	
}


//Intialize and configure UART2
void UART2Init(void)
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
	UART2_CTL_R &= ~(0x301); // 0001 Disable Uart by setting UARTEN=0 (bit 0 of UARTCTL)
	
	//Select system clock/PIOSC as UART Baud clock
	UART2_CC_R = 0x0; // (lower 4 bits) selct systemclock
	
	//Set Baud Rate
	// CLKDIV = 8(HSE=1) or 16(HSE=0) (HSE is bit5 in UARTCTL)
	// BRD = BRDI+BRDF = UARTSysClk/(ClkDiv*BaudRate)
	// BRD (16 bit) integer part of BRD
	// FBRD (6 bit) fractional part of BRD 
	// BRD = 16M/(16*9600) = 104.166
	UART2_CTL_R &= ~(0x20); // 0010_0000 HSE=0 for CLKDIV=16
	UART2_IBRD_R = 0x0068; 	// (lower 16 bits) IBRD=104 (in DIVINT field of UARTIBRD)
	UART2_FBRD_R = 0x0B; 		// (lower 6 bits) FBRD=integer(0.166666*64+0.5)=11 (in DIVFRAC field of UARTFBRD)
	
	//8bit word length,no parity bit,one stop bit,FIFOs enable
	UART2_LCRH_R &= ~(0x2); 		// 0010 Parity enable=0 (is bit1 in UARTLCRH)
	UART2_LCRH_R &= ~(0x8);  		// 1000 TwoStopBits=0 (is bit3 in UARTLCRH)
	UART2_LCRH_R |= 0x10;  	// 0001_0000 EnableFIFOs=1 (is bit4 in UARTLCRH)
	UART2_LCRH_R |= 0x40; 	// 0100_0000  WordLength=7 (bit5and6 in UARTLCRH)
	
	//Enable UART
	UART2_CTL_R |= 0x301; // 0001 Enable Uart by setting UARTEN=1 (bit 0 of UARTCTL)
	
}


//Wait for input and returns its ASCII value
unsigned char UART2Rx(void)
{
	while((UART2_FR_R & UART_FR_RX_FE)!=0);
	return((unsigned char)(UART2_DR_R & 0xFF));
}

//Output 7bit to serial port
void UART0Tx(unsigned char data)
{
	while((UART0_FR_R & UART_FR_TX_FF)!=0);
	UART0_DR_R=data;
}


/*Accepts ASCII characters from the serial port and
adds them to a string.It echoes each character as it
is inputted.*/
void UARTRxString (char *pt, unsigned short max)
{
	int len = 0;
	char char_;
	int a;
	char_ = UART2Rx();
	if(len < max){
		*pt = char_;
		pt++;
		len++;
		if (char_>='A' & char_<='Z'){
			UART0Tx(char_+32);
		}
		else if (char_>='a' & char_<='z'){
			UART0Tx(char_-32);
		}
		else{
			UART0Tx(char_);
		}

}
	*pt = 0;
}

//Output a character string to serial port
void UARTTxString(char *pt)
{
	while(*pt){
		UART0Tx(*pt);
		pt++;
	}
}

int main (void)
{
	char string[17];
	UART0Init();
	UART2Init();
	//The input given using keyboard is displayed on hyper terminal
	//.i.e.,data is echoed
	UARTTxString("EnterText:");

while(1)
	{
		UARTRxString(string,16);
	}
}
