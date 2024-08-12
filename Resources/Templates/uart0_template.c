/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: This is a template file for UART0 configuration. It echoes back
*				 the character sent.
 ******************************************************************************/


/******************************************************************************
*Macros for Register Addresses and Values
*******************************************************************************/
//Register definitions for ClockEnable
#define 	SYSCTL_RCGC_UART_R				(*((volatile unsigned long*)0x))
#define 	SYSCTL_RCGC_GPIO_R				(*((volatile unsigned long*)0x))

//Register definitions for GPIOPortA
#define 	GPIO_PORT_A_AFSEL_R				(*((volatile unsigned long*)0x))
#define 	GPIO_PORTA_PCTL_R				(*((volatile unsigned long*)0x))
#define 	GPIO_PORTA_DEN_R				(*((volatile unsigned long*)0x))
#define 	GPIO_PORTA_DIR_R				(*((volatile unsigned long*)0x))

//Register definitions for UART0_ module
#define 	UART0_CTL_R						(*((volatile unsigned long*)0x))
#define  	UART0_IBRD_R					(*((volatile unsigned long*)0x))
#define  	UART0_FBRD_R					(*((volatile unsigned long*)0x))
#define 	UART0_LCRH_R					(*((volatile unsigned long*)0x))
#define 	UART0_CC_R						(*((volatile unsigned long*)0x))
#define 	UART0_FR_R						(*((volatile unsigned long*)0x))
#define 	UART0_DR_R						(*((volatile unsigned long*)0x))

//Macros
#define 	UART_FR_TX_FF				0x20								//UART Transmit FIFO Full
#define 	UART_FR_RX_FE				0x10								//UART Receive FIFO Empty

//Function definitions
unsigned char UARTRx(void);
void UARTTx(unsigned char data);
void UARTTxString(char*pt);
void UARTRxString(char*bufPt, unsigned short max);

//Intialize and configure UART
void UARTInit(void)
{
	//Enable clock for UART0_ and GPIO PortA

	
	// Configure GPIO as UART (AFSEL,PCTL,DEN)

	
	//Disable UART
	
	
	//Select system clock/PIOSC as UART Baud clock
	
	
	//Set Baud Rate
	
	
	//8bit word length,no parity bit,one stop bit,FIFOs enable
	
	
	//Enable UART
	

}

//Wait for input and returns its ASCII value
unsigned char UARTRx(void)
{
	while((UART0_FR_R & UART_FR_RX_FE)!=0);
	return((unsigned char)(UART0_DR_R & 0x0FF));
}

/*Accepts ASCII characters from the serial port and
adds them to a string.It echoes each character as it
is inputted.*/
void UARTRxString (char *pt, unsigned short max)
{
	
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
	
}

int main(void)
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