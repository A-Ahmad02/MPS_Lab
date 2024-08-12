/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: 
*			
			GND () as GND (pin1)
			PE5 (U5Tx) as RXD (pin4) // PD7 (U2Tx) as RXD (pin4)
			VCC (3.3V) as VCC (pin5)
			PF4 (switch) as RST (pin6)
			PF2 (blue LED) as Chip_enable (pin7)
			PE4 (U5Rx) as TXD (pin8) // PD6 (U2Rx) as TXD (pin8)
			
		--PF1 (red LED) as GPIO0 for programming (pin3)
 ******************************************************************************/

#include "TM4C123.h"
#include "address.h"
#include <stdio.h>

//Macros
#define 	UART_FR_TX_FF				0x20								//UART Transmit FIFO Full
#define 	UART_FR_RX_FE				0x10								//UART Receive FIFO Empty
#define 	UART_FR_BUSY				0x08								//UART is busy
#define 	UART_RIS_RTRIS			0x40								//UART receive time out has occurred

/* Values to set */
#define NVIC_EN0_INT30 0x40000000 // 0100_0000_0000_0000_0000_0000_0000_0000 Interrupt 30 enable
#define GPIO_PORTF_CLK_EN 0x20		// (6 ports) 10_0000 to Clock enable for PortF
#define GPIO_PORTF_LEDS_EN 0x06		// (5 pins) 0_0100 to Enable LEDs (PF2)
#define GPIO_PORTF_SW1_EN 0x10		// (5 pins) 1_0000 to Enable user switch SW1 (PF4)
#define INT_PF4 0x10 							// (5 pins) 1_0000 as Interrupt at PF4

// BRD = 16M/(16*115,200) = 8.6805
#define IBRD 0x0008								// (lower 16 bits) IBRD=8 (in DIVINT field of UARTIBRD) integer part of BRD
#define FBRD 0x2C		 							// (lower 6 bits) FBRD=integer(0.6805*64+0.5)=44 (in DIVFRAC field of UARTFBRD) fractional part of BRD 

// BRD = 16M/(16*74,880) = 13.3547009
//#define IBRD 0x000D								// (lower 16 bits) IBRD=13 (in DIVINT field of UARTIBRD) integer part of BRD
//#define FBRD 0x17		 							// (lower 6 bits) FBRD=integer(0.3547*64+0.5)=23 (in DIVFRAC field of UARTFBRD) fractional part of BRD 

/* Global Variable */
char buffer[1000];
unsigned int i=0;

//Function definitions
void UART0Init (void); 
void UART2Init (void); 
void UART2Interrupt (void); 

void UART2Tx(unsigned char data);
unsigned char UART2Rx(void);

void UART0Tx(unsigned char data);
unsigned char UART0Rx(void);

void UARTTxString(char*pt);
void UART0TxString(char*pt);

/* Implements delay	*/
void Delay(unsigned long value);

/*-----These are functions added in startup file-----*/
/* Enable interrupts */
void EnableInterrupts(void);
/* Disableinterrupts */									
void DisableInterrupts(void);
/* Enable Priority interrupts */
void EnablePriorityInterrupts(void);
/* Implements WFI*/							
void WaitForInterrupt(void);


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
	UART0_CTL_R &= ~(0x1); // 0001 Disable Uart by setting UARTEN=0 (bit 0 of UARTCTL)
	
	//Select system clock/PIOSC as UART Baud clock
	UART0_CC_R = 0x0; // 0000 (lower 4 bits) select system as UART clock
	
	//Set Baud Rate
	// BRD = BRDI+BRDF = UARTSysClk/(ClkDiv*BaudRate)
	UART0_CTL_R &= ~(0x20); // 0010_0000 HSE=0 for CLKDIV=16 -> CLKDIV = 8(HSE=1) or 16(HSE=0) (HSE is bit5 in UARTCTL)
	UART0_IBRD_R = IBRD;
	UART0_FBRD_R = FBRD;
	
	//8bit word length,no parity bit,one stop bit,FIFOs enable
	UART0_LCRH_R &= ~(0x2); 		// 0010 Parity enable=0 (is bit1 in UARTLCRH)
	UART0_LCRH_R &= ~(0x8);  		// 1000 TwoStopBits=0 (is bit3 in UARTLCRH)
	UART0_LCRH_R |= 0x10;  	// 0001_0000 EnableFIFOs=1 (is bit4 in UARTLCRH)
	UART0_LCRH_R |= 0x60; 	// 0110_0000  WordLength=8 (bit5and6 in UARTLCRH)
	
	//Enable UART
	UART0_CTL_R |= 0x301; // 11_0000_0001 Enable Uart by setting UARTEN=1 (bit 0,8,9 of UARTCTL)
	
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
	GPIO_PORTD_CR_R |= 0x80;				// 1000_0000 (lower 8 bits) Unlock write protection
	
	// Configure GPIO as UART (AFSEL,PCTL,DEN)
	GPIO_PORTD_DEN_R |= 0xC0; 				// 1100_0000 enable pins PD6(Rx) and PD7(Tx)
	GPIO_PORTD_AFSEL_R |= 0xC0; 			// 1100_0000 Set(=1) enable alternate functionality PD6(Rx) and PD7(Tx)
	GPIO_PORTD_PCTL_R |= 0x11000000; 	// 0001_0001_0000_0000_0000_0000_0000_0000 port mux control for pins PD6 and PD7 (from Table 23-5)
	
	//Disable UART
	UART2_CTL_R &= ~(0x1); // 0001 Disable Uart by setting UARTEN=0 (bit 0 of UARTCTL)
	
	//Select system clock/PIOSC as UART Baud clock
	UART2_CC_R = 0x0; // (lower 4 bits) selct systemclock
	
	//Set Baud Rate
	// BRD = BRDI+BRDF = UARTSysClk/(ClkDiv*BaudRate)
	UART2_CTL_R &= ~(0x20); // 0010_0000 HSE=0 for CLKDIV=16 -> CLKDIV = 8(HSE=1) or 16(HSE=0) (HSE is bit5 in UARTCTL)
	UART2_IBRD_R = IBRD; 
	UART2_FBRD_R = FBRD; 		
	
	//8bit word length,no parity bit,one stop bit,FIFOs enable
	UART2_LCRH_R &= ~(0x2); 		// 0010 Parity enable=0 (is bit1 in UARTLCRH)
	UART2_LCRH_R &= ~(0x8);  		// 1000 TwoStopBits=0 (is bit3 in UARTLCRH)
	UART2_LCRH_R |= 0x10;  	// 0001_0000 EnableFIFOs=1 (is bit4 in UARTLCRH)
	UART2_LCRH_R |= 0x60; 	// 0110_0000  WordLength=8 (bit5and6 in UARTLCRH)
	
	//Enable UART
	UART2_CTL_R |= 0x301; // 11_0000_0001 Enable Uart, Tx and Rx by setting UARTEN=1 (bit 0, 8 and 9 of UARTCTL)
	
}


// Interrupt on Receive 
void UART2Interrupt (void){
	/* Globally disable all interrupts */
	DisableInterrupts();
	
	/* Enable interrupt in NVIC */
	// UART2 is IRQ33 -> EN1_INT1
	NVIC_EN1_R |= 0x2;		// 0000_0000_0000_0000_0000_0000_0000_0010 Interrupt 1 enable
	
	/* Set priority as 5 */
	// UART2 is IRQ33 -> PRI8_SET 1
	NVIC_PRI8_R = (NVIC_PRI8_R&0xFFFF00FF)|0x00008000 ; // 80 -> 1000_0000 as 100 is 4
		
	// Interrupt at 0x0 at bit5:3 RX FIFO = 1/8 full
	UART2_IFLS_R = (UART2_IFLS_R&0xFFFFFC7F); // C7 -> 1100_0111
	
	/* Disable all interrupts on UART2 */
	UART2_IM_R &= 0x00;	

	// Clear interrupt flag for UART2_RXIM at bit4 
	UART2_ICR_R |= 0x10;		// 0001_0000 Set as 1 to clear	
	
	// Enable interrupt on UART2_RXIM at bit4
	UART2_IM_R |= 0x10;			// 0001_0000 

	/* Enable interrupts beyond the defined priority level */
	EnablePriorityInterrupts();
	
	/* Globally enable all interrupts */
	EnableInterrupts();
	
}

/* Interrupt Service Routine for UART2 */
void UART2_Handler(void){						// Enter handler when FIFO 1/8 Full
		while((UART2_FR_R & UART_FR_RX_FE)==0){ // Loop till FIFO NOT empty
			buffer[i] = (UART2_DR_R & 0xFF);
			i++;
		}
		/* Clear interrupt flag */
		UART2_ICR_R |= 0x10;
}

//Intialize and configure Wifi Module
void WifiInit(void)
{
	/*	Set PF4 (switch) as reset
			Set PF1 (red LED) as GPIO0 for programming
			Set PF2 (blue LED) as Chip_enable
	*/
	volatile unsigned delay_clk;
	//Enable clock for UART2_ and respective GPIO Port
	SYSCTL_RCGC_GPIO_R |= 0x20; // 10_0000 to enable port F
	delay_clk = SYSCTL_RCGC_GPIO_R;
	
	
	/* Enable digital I/O on PF4,PF2, 
	* Make PF4 input and PF2 output
	* Enable weak pullup on PF4 */
	GPIO_PORTF_DEN_R |= (GPIO_PORTF_LEDS_EN|GPIO_PORTF_SW1_EN); 
	GPIO_PORTF_AFSEL_R &= ~(GPIO_PORTF_LEDS_EN|GPIO_PORTF_SW1_EN);
	GPIO_PORTF_DIR_R |= GPIO_PORTF_LEDS_EN;		// Set PF 2 as output (=1)
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
	
	// Enable Chip of Wifi Module PF2
	GPIO_PORTF_DATA_R |= 0x04;		// (5 pins) 0_0100 to Enable PF2
	
}


/* Interrupt Service Routine for PortF */
void GPIOF_Handler(void){
		
		/* Reset Wifi Module */
		GPIO_PORTF_DATA_R &= ~(INT_PF4);
		for(i=0;i<10000000;i++);
		GPIO_PORTF_DATA_R |= INT_PF4;
	
		/* Clear interrupt flag for PF4 */
		GPIO_PORTF_ICR_R |= INT_PF4;
}

//Wait for input and returns its ASCII value
unsigned char UART0Rx(void)
{
	while((UART0_FR_R & UART_FR_RX_FE)!=0);
	return((unsigned char)(UART0_DR_R & 0xFF));
}

//Output 7bit to serial port
void UART0Tx(unsigned char data)
{
	while((UART0_FR_R & UART_FR_TX_FF)!=0);
	UART0_DR_R=data;
	while (UART0_FR_R & UART_FR_BUSY);
}

//Wait for input and returns its ASCII value
unsigned char UART2Rx(void)
{
	while((UART2_FR_R & UART_FR_RX_FE)!=0);
	return (UART2_DR_R & 0xFF);
}

//Output 7bit to serial port
void UART2Tx(unsigned char data)
{
	while((UART2_FR_R & UART_FR_TX_FF)!=0);
	UART2_DR_R=data;
	while (UART2_FR_R & UART_FR_BUSY);

}


//Output a character string to serial port
void UARTTxString(char *pt)
{
	char *xt = pt;
	
	UART0Tx('S');
	UART0Tx('e');
	UART0Tx('n');
	UART0Tx('d');
	UART0Tx(':');
	UART0Tx(' ');
	
	while(*pt){
		UART0Tx(*pt);
		pt++;
	}
	UART0Tx(13);
	UART0Tx(10);
	
	while(*xt){
		//Delay(50000);
		UART2Tx(*xt);
		xt++;
	}
	UART2Tx(13);
	UART2Tx(10);
}
void UART0TxString(char *pt)
{
	while(i>0){
		UART0Tx(*pt);
		pt++;
		i--;
	}	
	UART0Tx(13);
	UART0Tx(10);
}

void Delay(unsigned long value){
	unsigned long k=0;
	for(k=0;k<value;k++);
}
int main (void)
{
	int delay = 10000000;
	char *str_ = "Bye\r\n";
	UART0Init();
	UART2Init();
	UART2Interrupt();
	WifiInit();
	
	UARTTxString("AT+RST");
	Delay(200000000);
	UART0TxString(buffer);
	
	UARTTxString("AT+CWMODE=3"); // Set the Wi-Fi mode=3 -> SoftAP+Station mode
	Delay(delay);
	UART0TxString(buffer);
	
	UARTTxString("AT+CWJAP=\"Infinix HOT 8\",\"qwerty00\""); // Connect to an AP -> ssid, password
	Delay(150000000);
	UART0TxString(buffer);
	
	UARTTxString("AT+CIFSR"); // Obtain the local IP address and MAC address.
	Delay(delay);
	UART0TxString(buffer);
	
	UARTTxString("AT+CIPMUX=1"); // Enable multiple connections mode.
	Delay(delay);
	UART0TxString(buffer);
	
	UARTTxString("AT+CIPSERVER=1,80"); // Mode=1 -> Create server, Port_number=80
	Delay(delay);
	UART0TxString(buffer);
	Delay(delay);
	UARTTxString("AT+CIPSTATUS"); // Obtain the TCP/UDP/SSL Connection Status and Information
	Delay(delay);
	UART0TxString(buffer);
	
	//UARTTxString("AT+CIPSEND=0,5");
	//Delay(500000);
	//UART0TxString(buffer);
	
	UARTTxString("AT+CIPSEND=0,5"); // Send Data in the Normal Transmission Mode
	Delay(delay);
	while(*str_){
			UART2Tx(*str_);
			str_++;
		}
	
	while(1){
		Delay(150000000);
		UART0TxString(buffer);
		}
}
