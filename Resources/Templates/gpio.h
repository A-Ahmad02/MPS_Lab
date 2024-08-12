/*********************************************************
* Macros for Register Addresses and Values
*********************************************************/

/*Register for clock*/
#define SYSCTL_RCGCGPIO_R (*(( volatile unsigned long *) 0x))

/*GPIO Registers for portB*/
#define GPIO_PORTB_DATA_R	(*(( volatile unsigned long *)0x))
#define GPIO_PORTB_DIR_R	(*(( volatile unsigned long *)0x))
#define GPIO_PORTB_DEN_R	(*(( volatile unsigned long *)0x))

/*GPIO Registers for portA*/
#define GPIO_PORTA_DATA_R (*(( volatile unsigned long *)0x))
#define GPIO_PORTA_DIR_R 	(*(( volatile unsigned long *)0x))
#define GPIO_PORTA_DEN_R 	(*(( volatile unsigned long *)0x))

/*Values for enabling seven segments*/
#define SEG_1 	0x
#define SEG_2 	0x
#define SEG_3 	0x
#define SEG_4 	0x
#define SEG_OFF 0xFF

/*Function Declarations*/
void init_gpio (void);
void display_1 (void);
void display_2 (void);
void delay (unsigned long value);