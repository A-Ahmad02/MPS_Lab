/******************************************************************************
*Macros for Register Addresses and Values
*******************************************************************************/
#define NVIC_EN0_R						(*((volatile unsigned long*)0xE000E100))
#define NVIC_PRI0_R						(*((volatile unsigned long*)0xE000E400))
#define NVIC_PRI1_R						(*((volatile unsigned long*)0xE000E404))
#define NVIC_PEND0_R					(*((volatile unsigned long*)0xE000E200))

#define GPIO_PORTB_DIR_R			(*((volatile unsigned long*)0x40005400))
#define GPIO_PORTB_DEN_R			(*((volatile unsigned long*)0x4000551C))
#define GPIO_PORTB_PUR_R			(*((volatile unsigned long*)0x40005510))
#define GPIO_PORTB_IS_R				(*((volatile unsigned long*)0x40005404))
#define GPIO_PORTB_IBE_R			(*((volatile unsigned long*)0x40005408))
#define GPIO_PORTB_IEV_R			(*((volatile unsigned long*)0x4000540C))
#define GPIO_PORTB_IM_R				(*((volatile unsigned long*)0x40005410))
#define GPIO_PORTB_ICR_R			(*((volatile unsigned long*)0x4000541C))

#define GPIO_PORTE_DIR_R			(*((volatile unsigned long*)0x40024400))
#define GPIO_PORTE_DEN_R			(*((volatile unsigned long*)0x4002451C))
#define GPIO_PORTE_PUR_R			(*((volatile unsigned long*)0x40024510))
#define GPIO_PORTE_IS_R				(*((volatile unsigned long*)0x40024404))
#define GPIO_PORTE_IBE_R			(*((volatile unsigned long*)0x40024408))
#define GPIO_PORTE_IEV_R			(*((volatile unsigned long*)0x4002440C))
#define GPIO_PORTE_IM_R				(*((volatile unsigned long*)0x40024410))
#define GPIO_PORTE_ICR_R			(*((volatile unsigned long*)0x4002441C))

/*-----These are functions added in startup file-----*/
/* Enable interrupts */
void EnableInterrupts(void);
/* Disableinterrupts */									
void DisableInterrupts(void);
/* Enable Priority interrupts */
void EnablePriorityInterrupts(void);
/* Implements WFI*/							
void WaitForInterrupt(void);
/* Initialize Interrupts */
void InterruptInit(void);
