/**********************************************************
*	Project Name: First Keil Project
*	File name: first_c
*	Author:
*	Date: 12-09-23
*	Description: Example in Exp 1 of MPS
*
**********************************************************/

#include "TM4C123.h"
void SystemInit (void)
{
/* --------------------------FPU settings ----------------------------------*/
#if (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2) |			/* set CP10 Full Access */
									(3UL << 11*2));			/* set CP11 Full Access */
#endif
}
	
int main ()
	{
		int a,b,c;
		a = 10;
		b = 15;
		c = a+b;
		return 0;
	}