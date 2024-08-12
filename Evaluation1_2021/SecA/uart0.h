/******************************************************************************
*Macros for Register Addresses and Values
*******************************************************************************/
//Register definitions for ClockEnable
#define 		SYSCTL_RCGC_UART_R			(*((volatile unsigned long*)0x400FE618))
#define 		SYSCTL_RCGC_GPIO_R			(*((volatile unsigned long*)0x400FE608))

//Register definitions for GPIOPortA
#define 	GPIO_PORT_A_AFSEL_R			(*((volatile unsigned long*)0x40004420))
#define 	GPIO_PORTA_PCTL_R				(*((volatile unsigned long*)0x4000452C))
#define 	GPIO_PORTA_DEN_R				(*((volatile unsigned long*)0x4000451C))
#define 	GPIO_PORTA_DIR_R				(*((volatile unsigned long*)0x40004400))

//Register definitions for UART0_ module
#define 	UART0_CTL_R						(*((volatile unsigned long*)0x4000C030))
#define  	UART0_IBRD_R					(*((volatile unsigned long*)0x4000C024))
#define  	UART0_FBRD_R					(*((volatile unsigned long*)0x4000C028))
#define 	UART0_LCRH_R					(*((volatile unsigned long*)0x4000C02C))
#define 	UART0_CC_R						(*((volatile unsigned long*)0x4000CFC8))
#define 	UART0_FR_R						(*((volatile unsigned long*)0x4000C018))
#define 	UART0_DR_R						(*((volatile unsigned long*)0x4000C000))

#define 	UART_FR_TX_FF				0x20								//UART Transmit FIFO Full
#define 	UART_FR_RX_FE				0x10								//UART Receive FIFO Empty

//Function definitions
unsigned char UARTRx(void);
void UARTTx(unsigned char data);
void UARTTxString(char*pt);
void UARTRxString(char*bufPt, unsigned short max);
void UARTInit(void);
