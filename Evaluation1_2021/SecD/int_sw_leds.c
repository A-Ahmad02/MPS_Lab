#include "ss_int_addr.h"
int roll [12] = {2,0,2,1,0xE,0xE,0,5,3,2,0,2} ;
static int dir;
const char lut_display[16]={0xC0,
							0xF9,
							0xA4,
							0xB0,
							0x99,
							0x92,
							0x82,
							0xF8,
							0x80,
							0x90,
							0x88,
							0x83,
							0xC6,
							0xA1,
							0x86,
							0x8E
							};

/* lut for segment selection */
const char seg_select[4]={0xDF,//SEG_1
						 0xEF,//SEG_2
						 0xF7,//SEG_3
						 0xFB//SEG_4
						};

void GPIOF_Handler(void){
	delay(10000);
	/* check which switch gave interrupt */
	if (GPIO_PORTF_MIS_R == (INT_PF0 & ~INT_PF4))
	{
		/* Clear interrupt flag for PF0 */
		GPIO_PORTF_ICR_R |= INT_PF0;
		GPIO_PORTD_DATA_R ^= 0x01;
		dir = 0;
	}
	
	if (GPIO_PORTF_MIS_R == (INT_PF4 & ~INT_PF0))
	{
		/* Clear interrupt flag for PF0 */
		GPIO_PORTF_ICR_R |= INT_PF4;
		GPIO_PORTD_DATA_R ^= 0x02;
		dir = 1;
	}
	
}

int main(){
	int i,k,j=0;
	SSinit();
	Interruptinit();
	while(1)
	{
		if (dir == 0)
		{
		for (k=0; k<100; k++)
		{
			//display number on seven segment display
		  for(i=0;i<4;i++)
		  {
			  GPIO_PORTA_DATA_R = 0x3C;
			  GPIO_PORTB_DATA_R = lut_display[roll[i+j]];
			  GPIO_PORTA_DATA_R = seg_select[i];
			  delay(10000);
      }
		}
		if (j>=8)
			j=0;
		else
	    j=j+1;
		
	}
	else if (dir == 1)
	{
		for (k=0; k<100; k++)
		{
			//display number on seven segment display
		  for(i=0;i<4;i++)
		  {
			  GPIO_PORTA_DATA_R = 0x3C;
			  GPIO_PORTB_DATA_R = lut_display[roll[i+j]];
			  GPIO_PORTA_DATA_R = seg_select[i];
			  delay(10000);
      }
		}
		if (j<=0)
			j=8;
		else
	    j=j-1;
	}
}
}
