;Lines to be added in startup file before Reset_Handler and after AREA    |.text|, CODE, READONLY        
		
                EXPORT  DisableInterrupts
                EXPORT  EnableInterrupts
                EXPORT  EnablePriorityInterrupts
                EXPORT  WaitForInterrupt
	
DisableInterrupts
                CPSID  I ; set PRIMASK to disable interrupts
                BX     LR

EnableInterrupts
                CPSIE  I ; clear PRIMASK to enable interrupts
                BX     LR

EnablePriorityInterrupts
                MOV     R0, #0x40 ; set base priority 2 
                MSR     BASEPRI, R0  
                BX      LR
        
WaitForInterrupt
                WFI
                BX     LR