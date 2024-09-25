/**
******************************************************************************
* @author	Junjae Lee
* @Mod		09-24-2024
* @brief	Embedded Controller:  LAB:SysTick
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"

int count = 0;
// Initialiization 
void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	GPIO_init(PC_13, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PC_13, EC_UP);//pull-up
	sevensegment_display_init(PA_7,PB_6,PC_7,PA_9);
	//SYS TICK INIT
	SysTick_init();
	
	//EXTI INIT
	EXTI_init(PC_13, FALL, 0);//falling edge, priority=0
}

int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_display(count);
		delay_ms(1000);
		count++;
		if (count >10) count =0;
		SysTick_reset();
	}
}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(PC_13)) {
		count=0;
		sevensegment_display(count);
		clear_pending_EXTI(PC_13); // cleared by writing '1'
	}
}