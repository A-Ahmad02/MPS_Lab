/******************************************************************************
*	Project name:
*	File name:
*	Author:
*	Date:
*	Description: This is a template file for writing C codes for first two lab sessions.
*	Also provides template for programs where floating point is used.
*	
 ******************************************************************************/

/* Adding empty SystemInit function allows for seeing main variables
	in debugger Call Stack + Locals Window */
	
void SystemInit (void){}

/* Main Function */
int main ()
{
	
	
	return 0;
}

/*	If need to use double or float values i.e. floating point unit is used,
	replace empty SystemInit function with this and also include the header file.*/
#include "TM4C123.h"

void SystemInit (void)
{
	  /* --------------------------FPU settings ----------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                  (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
}


