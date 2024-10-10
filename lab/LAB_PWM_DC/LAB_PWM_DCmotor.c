/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 10-02-2024
Modified         : 10-03-2024
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_PWM_DC_moter
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"
#include "math.h"


#define Moter_pin PA_0    
float duty=0.75;//duty ratio
void setup(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();

	while(1){}
}


// Initialization
void setup(void){
	
	RCC_PLL_init();				// System Clock = 84MHz
	GPIO_init(PC_13, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PC_13, EC_UP);//pull-up
	
	//Direct pin
	GPIO_init(PC_2, OUTPUT); //Calls RCC_GPIO_enable(), output
	GPIO_otype(PC_2, 0);//push-pull
	
	//EXTI INIT
	EXTI_init(PC_13, FALL, 0);//falling edge, priority=0
	
	//1 = 1msec
	PWM_init(Moter_pin);
	
	// this lab push pull, pull-up, fast
	GPIO_pupd(Moter_pin,1);//pull-up
	GPIO_ospeed(Moter_pin,Fast_speed);//fast
	
	//1= 1msec
	PWM_period(Moter_pin,1);
	PWM_duty(LED_pin,duty);//change duty

	
	//1= 1msec
	
	TIM_UI_init(TIM3, 500);			// TIM3 Update-Event Interrupt every 500 msec 
	TIM_UI_enable(TIM3); //TIM3 enable

	
	

}
int state=0;//0->up, 1->down
uint32_t _count = 0; //to change 500msec

void TIM3_IRQHandler(void){
	float change_duty;//to change duty ratio, 0.25->0.75->0.25...

	if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
		_count++;
		if(_count>4){//change every 2 second
			//consider dir
			//float targetPWM;  // pwm for motor input 
			//float duty=abs(DIR-targetPWM); // duty with consideration of DIR=1 or 0
		change_duty=duty;//current duty
		duty=fabs(1.0-change_duty);//calculate change duty
		PWM_duty(Moter_pin,duty);//change duty
			_count=0;//reset count
	}
		
		clear_UIF(TIM3); 		// Clear UI flag by writing 0
	}
}


int count=0;
//button intterupt
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(PC_13)) {//check PC_13 is pending
		TIM2->CR1^=1;//TOGGLE enable clock
		clear_pending_EXTI(PC_13); // cleared by writing '1'
	}
}
