/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: 
*			
			ESP-01
			GND () as GND (pin1)
			PD7 (U2Tx) as RXD (pin4)
			VCC (3.3V) as VCC (pin5)
			PF4 (switch) as RST (pin6)
			PF2 (blue LED) as Chip_enable (pin7)
			PD6 (U2Rx) as TXD (pin8)
		
			MPU
			GND as GND
			VCC (3.3V) as VCC (2.375V-3.46V)
			PB2 (I2C0SCL) as SCL
			PB3 (I2C0SDA) as SDA
			PF2 (blue LED) as Recieving/Acknowleged
			
		--I2C ADDRESS AD0 = 0 -> 1101000 -> 0x68
									AD0 = 1 -> 1101001 -> 0x69
									
		--I2CM_CONTROL_R is the write only of I2C0_MCS_R
			I2C0_STATUS_R is the read only of I2C0_MCS_R
 ******************************************************************************/

#include "TM4C123.h"
#include "address.h"
#include <stdio.h>
#include <stdlib.h>

// Macros for I2C
// I2C bit field definitions
# define I2C_ENABLE 						0x01
# define BUSY_STATUS_FLAG 			0x01
// I2C command definitions for control register
# define CMD_SINGLE_SEND 				0x7				// 0111
# define CMD_SINGLE_RECEIVE 		0x7				// 0111
# define CMD_BURST_SEND_START 	0xB				// 1011
# define CMD_BURST_SEND_CONT 		0x9				// 1001
# define CMD_BURST_SEND_FINISH 	0x5				// 0101

// Pg 1010 Fig 16-10 Master RECEIVE of Multiple Data Bytes
# define CMD_BURST_RECEIVE_START 	0xB			// 1011 -> X011
# define CMD_BURST_RECEIVE_CONT 	0x9			// 1001 -> X001
# define CMD_BURST_RECEIVE_FINISH 0x5			// 0101 -> X101
//
# define SLAVE_ADDR							0x68			// AD0=0 -> MPU6050 is 0x68

//Macros for UART
#define 	UART_FR_TX_FF					0x20				//UART Transmit FIFO Full
#define 	UART_FR_RX_FE					0x10				//UART Receive FIFO Empty

// Macros for Interrupts
#define NVIC_EN0_INT30 					0x40000000 	// 0100_0000_0000_0000_0000_0000_0000_0000 Interrupt 30 enable
#define GPIO_PORTF_CLK_EN 			0x20				// (6 ports) 10_0000 to Clock enable for PortF
#define GPIO_PORTF_LEDS_EN 			0x06				// (5 pins) 0_0100 to Enable LEDs (PF2)
#define GPIO_PORTF_SW1_EN 			0x10				// (5 pins) 1_0000 to Enable user switch SW1 (PF4)
#define INT_PF4 								0x10 				// (5 pins) 1_0000 as Interrupt at PF4

// BRD = 16M/(16*115,200) = 8.6805
#define IBRD 										0x0008			// (lower 16 bits) IBRD=8 (in DIVINT field of UARTIBRD) integer part of BRD
#define FBRD 										0x2C		 		// (lower 6 bits) FBRD=integer(0.6805*64+0.5)=44 (in DIVFRAC field of UARTFBRD) fractional part of BRD 


//constant values for Timers
#define		TIM1_CLK_EN             0x02
#define		TIM1_EN                 0x01

#define		TIM_16_BIT_EN           0x04
#define		TIM_32_BIT_EN           0x00

#define		TIM_TAMR_PERIODIC_EN    0x02
#define		TIM_TAMR_ONESHOT_EN    0x01
#define		TIM1_INT_CLR            0x01		// Interrupt GPTM Timer A 

#define		EN0_INT21               0x200000 // 16/32-Bit Timer 1A

#define		PORTF_CLK_EN            0x20
#define		TOGGLE_PF1              0x02
#define		LED_RED                 0x02	


/* Global Variable */
char buffer2[1000];
unsigned int i2=0;
unsigned char buffer[14];
int count;

//Function definitions
void UART0Init (void); 
void UART2Init (void); 
void MPU6050Init (void); 
void I2C0MInit (void); 

void GPIO_Init(void);
void Timer_Init(unsigned long period);

void I2C_Receive_Data_14(unsigned char Data_addr);
void I2C_Send_Data(unsigned char Data_addr, unsigned char Data);

void UART0Tx(unsigned char data);
unsigned char UART0Rx(void);
void UART2Tx(unsigned char data);
unsigned char UART2Rx(void);


void UARTRxString(char*pt);
void UARTTxString(char*pt);
void UART0TxString(void);
void UARTTxString_MPU(char *pt);

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


void SystemInit (void){
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}
	


//Intialize and configure UART0
void UART0Init(void){
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
void UART2Init(void){
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

//Intialize and configure Wifi Module
void WifiInit(void){
	/*	Set PF4 (switch) as reset
			Set PF1 (red LED) as 
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
	
	
		/* Reset Wifi Module */
		GPIO_PORTF_DATA_R &= ~(INT_PF4);
		for(i2=0;i2<100;i2++);
		GPIO_PORTF_DATA_R |= INT_PF4;
	
}



/* Interrupt Service Routine for PortF */
void GPIOF_Handler(void){
		
		/* Reset Wifi Module */
		GPIO_PORTF_DATA_R &= ~(INT_PF4);
		for(i2=0;i2<10000000;i2++);
		GPIO_PORTF_DATA_R |= INT_PF4;
	
		/* Clear interrupt flag for PF4 */
		GPIO_PORTF_ICR_R |= INT_PF4;
}


//Intialize and configure I2C0 as Master
void I2C0MInit(void){
	volatile unsigned delay_clk;
	//Enable clock for UART2_ and respective GPIO Port
	SYSCTL_RCGC_GPIO_R |= 0x02; // 00_0010 to enable port B
	delay_clk = SYSCTL_RCGC_GPIO_R;
	SYSCTL_RCGC_I2C_R |= 0x1; // 0001 to enable I2C 0 (I2C 0-3)
	delay_clk = SYSCTL_RCGC_I2C_R;
	
	// Configure GPIO as UART (AFSEL,PCTL,DEN)
	GPIO_PORTB_DEN_R |= 0x0C; 				// 0000_1100 enable pins PB2(SCL) and PB3(SDA)
	GPIO_PORTB_AFSEL_R |= 0x0C; 			// 0000_1100 Set(=1) enable alternate functionality PB2(SCL) and PB3(SDA)
	GPIO_PORTB_PCTL_R |= 0x00003300; 	// 0000_0000_0000_0000_0011_0011_0000_0000 port mux control for pins  PB2 and PB3(from Table 23-5)
	GPIO_PORTB_OD_R |= 0x8 ; 					// 0000_1000 SDA (PB3) pin as open drain
	
	I2C0_MCR_R = 0x0010 ; // Enable I2C 0 master function (bit 4)
	
	/* Configure I2C 0 clock frequency
	(1 + TIME_PERIOD ) = SYS_CLK /(2*( SCL_LP + SCL_HP ) * I2C_CLK_Freq )
	SYS_CLK -> 16MHz and SCL_LP -> 6 and SCL_HP -> 4
	I2C_CLK_Freq -> 100k, 400k, 1M, 3.33M
	TIME_PERIOD = 16,000,000/(2(6+4)*100,000)-1 = 7 */
	I2C0_MTPR_R = 0x07 ; // bits 6:0
	
}

//Intialize and configure MPU6050
void MPU6050Init(void){
	// Wait for the master to become idle
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	
	// Configure slave address for write operation , data and control
	I2C0_MSA_R |= ( SLAVE_ADDR << 1);
	
	// Configure for read operation from the slave address 
	I2C0_MSA_R |= 0x01 ; // 1 -> recieve (bit 0) 
	
}

// Receive one byte of data from I2C slave device
void I2C_Receive_Data_14 (unsigned char Data_addr) {
	int n = 0;
	
		// Wait for the master to become idle
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	
	// Configure slave address for write operation , data and control
	I2C0_MSA_R = ( SLAVE_ADDR << 1);
	I2C0_MDR_R = Data_addr ; // Data to be written
	I2C0_MCS_R = CMD_SINGLE_SEND ; // Command to be performed
	
	// Wait until master is done
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	// Configure the slave address for read operation
	I2C0_MSA_R = (( SLAVE_ADDR << 1) | 0x01 );
	I2C0_MCS_R = CMD_BURST_RECEIVE_START ; // Command to be performed
	
	while ( I2C0_MCS_R & 0x03 );
	
	while(n < 14){
		buffer[n] = (unsigned char) I2C0_MDR_R;
		//if (n ==13){break;}
		I2C0_MCS_R = CMD_BURST_RECEIVE_CONT ;
		while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
		n++;
	}
	// Wait until master is done
	I2C0_MCS_R = CMD_BURST_RECEIVE_FINISH ;
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	return ; // Read the data received

}
void I2C_Send_Data ( unsigned char Data_addr , unsigned char Data ) {
	// Wait for the master to become idle
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	
	// Configure slave address for write operation , data and control
	I2C0_MSA_R = ( SLAVE_ADDR << 1);
	I2C0_MDR_R = Data_addr ; // Data to be written
	I2C0_MCS_R = CMD_BURST_SEND_START ; // Command to be performed
	
	// Wait until master is done
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	
	// Configure data to be written and send command
	I2C0_MDR_R = Data ;
	I2C0_MCS_R = CMD_BURST_SEND_CONT ;
	
	// Wait until master is done
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	
	// Send the finsh command
	I2C0_MCS_R = CMD_BURST_SEND_FINISH ;
	
	// Wait until master is done
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
}
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
}
//Wait for input and returns its ASCII value
unsigned char UART2Rx(void){
	while((UART2_FR_R & UART_FR_RX_FE)!=0);
	return ( (unsigned char)(UART2_DR_R & 0xFF) );
}
//Output 7bit to serial port
void UART2Tx(unsigned char data){
	while((UART2_FR_R & UART_FR_TX_FF)!=0);
	UART2_DR_R=data;
}
void UARTTxString_MPU(char *pt)
{	
	while(*pt){
		UART0Tx(*pt);
		pt++;
	}
}
//Output a character string to serial port
void UARTTxString(char *pt){
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
		UART2Tx(*xt);
		xt++;
	}
	UART2Tx(13);
	UART2Tx(10);
}

void UART0TxString(void){
	int a = 0;
	while(a < i2){
		UART0Tx(buffer2[a]);
		a++;
	}	
	i2 = 0;
	UART0Tx(13);
	UART0Tx(10);
}
void UARTRxString(char *pt)
{	
	char *xt = pt;
	// Loop until maximum characters or next line char
	while(*xt){
		buffer2[i2] = UART2Rx();
		if(buffer2[i2] == *xt){
			xt++;
		}
		else{
			xt = pt;
		}	
		i2++;
	}
	UART0TxString();
}

void Timer_Init(unsigned long period)
{
	//enable clock for Timer1
	SYSCTL->RCGCTIMER |= TIM1_CLK_EN ;	// Turn on Timer 1
	
	//disable Timer1 before setup
	TIMER1->CTL  &= ~(TIM1_EN);		// Disable Timer 1							
	//configure 32-bit timer mode
	TIMER1->CFG  |= (TIM_32_BIT_EN);		// Use whole timer
	//configure periodic mode
	TIMER1->TAMR = TIM_TAMR_PERIODIC_EN;	// Enable Periodic same for TA and wholeTimer 	
	//set initial load value
	TIMER1->TAILR = period;	
	//set prescalar for desired frequency 16M
	TIMER1->TAPR  = 0;	
	
	DisableInterrupts();
	//Set priority for interrupt
	NVIC_SetPriority(TIMER1A_IRQn,2);
	//enable interrupt 21 (timer1A interrupt)												
	NVIC_EnableIRQ(TIMER1A_IRQn);
	//clear timeout interrupt
	TIMER1->ICR  |= TIM1_INT_CLR;										
	//enable interrupt mask for Timer0A
	TIMER1->IMR  |= TIM1_INT_CLR;												
	
	//enableTimer0A
	TIMER1->CTL  |= TIM1_EN;												
	
	EnableInterrupts();
}


void GPIO_Init()
{
	//configure red led (PF1)
	SYSCTL->RCGCGPIO |= PORTF_CLK_EN;
	GPIOF->DEN       |= LED_RED;
	GPIOF->DIR       |= LED_RED;
}


void TIMER1A_Handler(void)
{
	TIMER1->ICR   = TIM1_INT_CLR;
}

void Delay(unsigned long value){
	unsigned long k=0;
	for(k=0;k<value;k++);
}

int main (void){
	char c[50];
	char* str_;
	float Ax, Ay, Az;
	float Gx, Gy, Gz;
	
	UART0Init();
	UART2Init();
	WifiInit();
	I2C0MInit();
	
	I2C_Send_Data (0x6B,0x00); // Write 0x00 to 0x6B to enable MPU by disabling reset
	I2C_Send_Data (0x1C,0x10); // Configure the accelerometer (+/-8g)
	I2C_Send_Data (0x1B,0x08); // Configure the gyro (500dps full scale)

	UARTTxString("AT+RST");
	UARTRxString("ready");

	UARTTxString("AT+CWMODE=3"); // Set the Wi-Fi mode=3 -> SoftAP+Station mode
	UARTRxString("OK");
	
	// Additional instructions for ESP-01
  UARTTxString("AT+CWSAP=\"ESP_12D9F2\",\"12345678\",1,3"); // Set SoftAP parameters (SSID, password, channel, encryption)
  UARTRxString("OK");
		
  UARTTxString("AT+CIFSR"); // Obtain the local IP address and MAC address.
	UARTRxString("OK");
	
	UARTTxString("AT+CIPMUX=1"); // Enable multiple connections mode.
	UARTRxString("OK");
	
	
	UARTTxString("AT+CIPSERVER=1,80"); // Mode=1 -> Create server, Port_number=80
	UARTRxString("OK");

 // Wait for the WiFi server to establish connection
	// This might take around 5-10 seconds
	//Delay(10000000); // Adjust the delay as needed

	// Indicate the user to press the EN button on ESP32
	// For example, turn on an LED
	GPIO_PORTF_DATA_R |= 0x02; // Assuming PF1 is connected to an LED
	Timer_Init(80000000);		
	while ((TIMER1->RIS & 0x00000001) == 0);
	TIMER1->CTL  &= ~(TIM1_EN);		// Disable Timer 1	
	GPIO_PORTF_DATA_R &= ~0x02; // Turn off the LED

	UARTTxString("AT+CIPSTATUS"); // Obtain the TCP/UDP/SSL Connection Status and Information
	UARTRxString("OK");
	
	while(1){
		//Delay(5000000);
		Delay(50000);
		
		I2C_Receive_Data_14(0x3B);
		Ax = ((int16_t)(((buffer[0])<<8|(buffer[1])))/4096.0);
		Ay = ((int16_t)(((buffer[2])<<8|(buffer[3])))/4096.0);
		Az = ((int16_t)(((buffer[4])<<8|(buffer[5])))/4096.0);
		Gx = ((int16_t)(((buffer[8])<<8|(buffer[9])))/65.5);
		Gy = ((int16_t)(((buffer[10])<<8|(buffer[11])))/65.5);
		Gz = ((int16_t)(((buffer[12])<<8|(buffer[13])))/65.5);
		
		UARTTxString_MPU(" Ax= ");
		sprintf(c, "%g", Ax); // Typecast float to string
		UARTTxString_MPU(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString_MPU(" Ay= ");
		sprintf(c, "%g", Ay);
		UARTTxString_MPU(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString_MPU(" Az= ");
		sprintf(c, "%g", Az);
		UARTTxString_MPU(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		
		UARTTxString_MPU("   Gx= ");
		sprintf(c, "%f", Gx);
		UARTTxString_MPU(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString_MPU(" Gy= ");
		sprintf(c, "%g", Gy);
		UARTTxString_MPU(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString_MPU(" Gz= ");
		sprintf(c, "%g", Gz);
		UARTTxString_MPU(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		
		UART0Tx(13);
		UART0Tx(10);
		if (Ax>=-0.5 & Ax<=0.5 & Ay>=-0.5 & Ay<=0.5 & Az>=1.5 & Az<=2){
			UARTTxString("stop\r\n");
			str_ = "stop";
			UARTTxString("AT+CIPSEND=0,4");
		}
		else if (Ax>=0 & Ax<=1 & Ay>=1.5 & Ay<=2.5 & Az>=-0.5 & Az<=0.5){
			UARTTxString("right\r\n");
			str_ = "right";
			UARTTxString("AT+CIPSEND=0,5");
		}
		else if (Ax>=-1 & Ax<=0 & Ay>=-2.5 & Ay<=-1.5 & Az>=-0.5 & Az<=0.5){
			UARTTxString("left\r\n");
			str_ = "left";
			UARTTxString("AT+CIPSEND=0,4");
		}
		else if (Ax>=-2.5 & Ax<=0 & Ay>=-0.6 & Ay<=0.6 & Az>=-0.7 & Az<=0.5){
			UARTTxString("front\r\n");
			str_ = "front";
			UARTTxString("AT+CIPSEND=0,5");
		}
		else if (Ax>=0 & Ax<=2.5 & Ay>=-0.6 & Ay<=0.6 & Az>=-0.5 & Az<=0.7){
			UARTTxString("back\r\n");
			str_ = "back";
			UARTTxString("AT+CIPSEND=0,4");
		}
	while(*str_){
		UART2Tx(*str_);
		str_++;
	}
 }
}
