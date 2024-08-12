/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: 
*			
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

/* Global Variable */
unsigned char buffer[14];

//Function definitions
void UART0Init (void); 
void MPU6050Init (void); 
void I2C0MInit (void); 
void I2C0Interrupt (void); 

void I2C_Receive_Data_14(unsigned char Data_addr);
unsigned char I2C_Receive_Data (unsigned char Data_addr);
void I2C_Send_Data(unsigned char Data_addr, unsigned char Data);

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



// Interrupt on Receive 
void I2C0Interrupt (void){
	/* Globally disable all interrupts */
	DisableInterrupts();
	
	/* Enable interrupt in NVIC */
	// I2C0 is IRQ8 -> EN0_INT8
	NVIC_EN0_R |= 0x100;		// 0000_0000_0000_0000_0000_0001_0000_0000 Interrupt 0 enable
	
	/* Set priority as 5 */
	// I2C0 is IRQ8 -> PRI2_SET 0
	NVIC_PRI2_R = (NVIC_PRI2_R&0xFFFFFF00)|0x00000080 ; // 80 -> 1000_0000 as 100 is 4
		
	/* Disable all interrupts on I2C0 */
	I2C0_MIMR_R &= 0x0;	

	// Clear interrupt flag for UART2_RXIM at bit4 
	I2C0_MICR_R |= 0x1;		// 01 Set as 1 to clear	
	
	// Enable interrupt on I2C0
	I2C0_MIMR_R |= 0x1;			// 01 -> Master Interrupt Mask (bit0)

	/* Enable interrupts beyond the defined priority level */
	EnablePriorityInterrupts();
	
	/* Globally enable all interrupts */
	EnableInterrupts();
	
}


/* Interrupt Service Routine for I2C0 */
void I2C0_Handler(void){	

		/* Clear interrupt flag */
		I2C0_MICR_R |= 0x1;
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
unsigned char I2C_Receive_Data (unsigned char Data_addr) {
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
	I2C0_MCS_R = CMD_SINGLE_RECEIVE ; // Command to be performed
	
	// Wait until master is done
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	return ( unsigned char ) I2C0_MDR_R ; // Read the data received

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
void UARTTxString(char *pt)
{	
	while(*pt){
		UART0Tx(*pt);
		pt++;
	}

}

void Delay(unsigned long value){
	unsigned long k=0;
	for(k=0;k<value;k++);
}

int main (void){
	char c[50];
	int16_t Ax_offset=0;
	int16_t Ay_offset=0;
	int16_t Az_offset=0;
	int16_t Gx_offset=0;
	int16_t Gy_offset=0;
	int16_t Gz_offset=0;
	int i=0;
	
	UART0Init();
	I2C0MInit();
	//I2C0Interrupt();
	//MPU6050Init();
	I2C_Send_Data (0x6B,0x00); // Write 0x00 to 0x6B to enable MPU by disabling reset
	I2C_Send_Data (0x1C,0x10); // Configure the accelerometer (+/-8g)
	I2C_Send_Data (0x1B,0x08); // Configure the gyro (500dps full scale)
	//UARTTxString("Ax= ");
	//UART0Tx(I2C_Receive_Data(0x3B)<<8|I2C_Receive_Data(0x3C)); // ACCEL_XOUT[15:8]
	while(i<10){
		I2C_Receive_Data_14(0x3B);
		Ax_offset += ((buffer[0])<<8|(buffer[1]));
		Ay_offset += ((buffer[2])<<8|(buffer[3]));
		Az_offset += ((buffer[4])<<8|(buffer[5]));
		Gx_offset += ((buffer[8])<<8|(buffer[9]));
		Gy_offset += ((buffer[10])<<8|(buffer[11]));
		Gz_offset += ((buffer[12])<<8|(buffer[13]));

		i++;
	}
	Ax_offset /= 10;
	Ay_offset /= 10;
	Az_offset /= 10;
	Gx_offset /= 10;
	Gy_offset /= 10;
	Gz_offset /= 10;
	while(1){
		Delay(500000);
		I2C_Receive_Data_14(0x3B);

		UARTTxString(" Ax= ");
		sprintf(c, "%g", ((int16_t)(((buffer[0])<<8|(buffer[1]))-Ax_offset)/4096.0)); // Typecast float to string
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString(" Ay= ");
		sprintf(c, "%g", ((int16_t)(((buffer[2])<<8|(buffer[3]))-Ay_offset)/4096.0));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString(" Az= ");
		sprintf(c, "%g", ((int16_t)(((buffer[4])<<8|(buffer[5]))-Az_offset)/4096.0));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		
		UARTTxString("   Gx= ");
		sprintf(c, "%f", ((int16_t)(((buffer[8])<<8|(buffer[9]))-Gx_offset)/65.5));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString(" Gy= ");
		sprintf(c, "%g", ((int16_t)(((buffer[10])<<8|(buffer[11]))-Gy_offset)/65.5));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString(" Gz= ");
		sprintf(c, "%g", ((int16_t)(((buffer[12])<<8|(buffer[13]))-Gz_offset)/65.5));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		
		UART0Tx(13);
		UART0Tx(10);
		
		/*
		UARTTxString(" Ax= ");
		sprintf(c, "%g", ((int16_t)((buffer[0])<<8|(buffer[1]))/4096.0)); // Typecast float to string
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString(" Ay= ");
		sprintf(c, "%g", ((int16_t)((buffer[2])<<8|(buffer[3]))/4096.0));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString(" Az= ");
		sprintf(c, "%g", ((int16_t)((buffer[4])<<8|(buffer[5]))/4096.0));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		
		UARTTxString("   Gx= ");
		sprintf(c, "%f", ((int16_t)((buffer[8])<<8|(buffer[9]))/65.5));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString(" Gy= ");
		sprintf(c, "%g", ((int16_t)((buffer[10])<<8|(buffer[11]))/65.5));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		UARTTxString(" Gz= ");
		sprintf(c, "%g", ((int16_t)((buffer[12])<<8|(buffer[13]))/65.5));
		UARTTxString(c); // ACCEL_XOUT[15:8] and ACCEL_XOUT[7:0]
		
		UART0Tx(13);
		UART0Tx(10);*/
		
		}
}




/*
// I2CM_CONTROL_R is the write only of I2C0_MCS_R
// I2C0_MCS_R is the read only of I2C0_MCS_R

// Receive one byte of data from I2C slave device
unsigned char I2C_Receive_Data ( unsigned char Slave_addr , unsigned char Data_addr ) {
	unsigned char value = 0;
	// Wait for the master to become idle
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	
	// Configure slave address for write operation , data and control
	I2C0_MSA_R = ( Slave_addr << 1);
	I2C0_MDR_R = Data_addr ; // Data to be written
	I2C0_MCS_R = CMD_SINGLE_SEND ; // Command to be performed
	
	// Wait until master is done
	while ( I2C0_MCS_R & 0 x01 );
	// Configure the slave address for read operation
	I2C0_MSA_R = (( Slave_addr << 1) | 0x01 );
	I2C0_MCS_R = CMD_SINGLE_RECEIVE ; // Command to be performed
	
	// Wait until master is done
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	value = ( unsigned char ) I2C0_MDR_R ; // Read the data received
	return value ;
}

// This function writes one data byte to Data_addr of I2C slave
void I2C_Send_Data ( unsigned char Slave_addr ,
unsigned char Data_addr , unsigned char Data ) {
	// Wait for the master to become idle
	while ( I2C0_MCS_R & BUSY_STATUS_FLAG );
	
	// Configure slave address for write operation , data and control
	I2C0_MSA_R = ( Slave_addr << 1);
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
*/
