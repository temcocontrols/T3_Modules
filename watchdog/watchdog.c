#include "watchdog.h"


void watchdog_init(void)
{
		/* Enable write access to IWDG_PR and IWDG_RLR registers */ 
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		/* IWDG counter clock: 40KHz(LSI) / 4 = 10 KHz */ 
		IWDG_SetPrescaler(IWDG_Prescaler_64); 
		/* Set counter reload value to 10000 = 1s */ 
		IWDG_SetReload(4000); 
		IWDG_ReloadCounter(); // reload the value
		IWDG_Enable();  			//enable the watchdog

}


