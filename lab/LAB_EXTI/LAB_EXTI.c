/**
******************************************************************************
* @author	Junjae Lee
* @Mod		09-24-2024
* @brief	Embedded Controller:  LAB:EXTI
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
	RCC_PLL_init();//PLL init
	SysTick_init(1000);//Systick init
	GPIO_init(PC_13, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PC_13, EC_UP);//pull-up
	sevensegment_display_init(PA_7,PB_6,PC_7,PA_9);//use PA_7:D, PB_6:C, PC_7:B, PA_9:A
	EXTI_init(PC_13, FALL, 0);//falling edge, priority=0
}

int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1);//use interrupt method
}

void EXTI15_10_IRQHandler(void) {

	if (is_pending_EXTI(PC_13)) {//check PC_13 is pending
		count=(count+1)%10;//increase count
		sevensegment_display(count);//display number
		clear_pending_EXTI(PC_13); // cleared by writing '1'
	}
}