/**********************************************************
*	Project Name: ADC Lab Experiment
*	File name: Ex2-0
*	Author:
*	Date: 12-09-23
*	Description: Example 0 of Exp 2
*
**********************************************************/

// Including files
// #include "lm4f120h5qr.h"
#include <stdio.h>

// Object like macros
#define identifier replacement
#define DELAY 20000

// Function like macros
#define SUM(a,b,c ) a + b + c
#define SQR(c) ((c) * (c))
	
void SystemInit (void)
{
/* --------------------------FPU settings ----------------------------------*/
#if (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2) |			/* set CP10 Full Access */
								(3UL << 11*2	);			/* set CP11 Full Access */
#endif
}
	
int main ()
	{
		char c;
		int b;
		b = SQR(DELAY);
		printf ( " C Programming for Embedded Systems \n " ) ;
		c = getchar( ) ;
		return 0;
	}