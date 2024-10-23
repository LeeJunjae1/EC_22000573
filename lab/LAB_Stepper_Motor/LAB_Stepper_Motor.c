/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 10-22-2024
Modified         : 10-22-2024
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_Stepper_Motor
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecEXTI2.h"
#include "ecSysTick2.h"
#include "ecStepper2.h"

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	Stepper_step(2048*2, 0, FULL);  // (Step : 2048, Direction : 0 or 1, Mode : FULL or HALF)
	//1 cw
	//0 ccw
	//step 2048(1 rotation, 1 rpm)
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){;}
}

// Initialiization 
void setup(void){
	
	RCC_PLL_init();                                 // System Clock = 84MHz
	SysTick_init(1000);                                 // Systick init
	
	GPIO_init(BUTTON_PIN, INPUT);  	           // GPIOC pin13 initialization
	EXTI_init(BUTTON_PIN, FALL,0);           // External Interrupt Setting


	Stepper_init(PB_10,PB_4,PB_5,PB_3); // Stepper GPIO pin initialization
	Stepper_setSpeed(2);                          	//  set stepper motor speed
	// max: 14rpm, min: 1rpm (full)
	// max: 29(14.5rpm), min: 1(0.5rpm) (half)
}

void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)) {
		Stepper_stop();
		delay_ms(1);
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}