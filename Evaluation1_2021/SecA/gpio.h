#define GPIO_PORTF_DATA_R (*((volatile unsigned long*) 0x400253FC))
#define GPIO_PORTF_DIR_R (*((volatile unsigned long*) 0x40025400))
#define GPIO_PORTF_DEN_R (*((volatile unsigned long*) 0x4002551C))

#define GPIO_PORTF_CLK_EN 0x20
#define GPIO_PORTF_PIN3_EN 0x0C
	/* Values to be entered in the registers */
#define LED_ON 0x08
#define LED_OFF ~(0x08)
#define DELAY 800000
	
void GPIOinit ( void );
