/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 10-02-2024
Modified         : 10-03-2024
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_PWM_RC_moter
/----------------------------------------------------------------*/



#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"


#define Moter_pin PA_1    
float pulse_width=0.5;//pulse width
void setup(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();
	while(1){}
}


// Initialization
void setup(void){
	

	//TIM IN tu
	RCC_PLL_init();				// System Clock = 84MHz
	GPIO_init(PC_13, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PC_13, EC_UP);//pull-up
	
	//EXTI INIT
	EXTI_init(PC_13, FALL, 0);//falling edge, priority=0
	
	//100 = 1msec
	PWM_init(Moter_pin);//initialize pwm
	
	// this lab push pull, pull-up, fast
	GPIO_pupd(Moter_pin,1);//pull-up
	GPIO_ospeed(Moter_pin,Fast_speed);//fast
	
	PWM_period(Moter_pin,2000);//period 20msec
	PWM_pulsewidth(Moter_pin,pulse_width);//to change pulse width
	
	//100 = 1msec
	TIM_UI_init(TIM3, 100);			// TIM3 Update-Event Interrupt every 1 msec 
	TIM_UI_enable(TIM3); //TIM3 enable
	

}
int state=0;//0->up, 1->down
uint32_t _count = 0;//to change 500msec
uint32_t degree = 0;//to change rotation


void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
		_count++;
		if(_count>500){//change 500msec
			if(state==0){//up state
				degree++;//plus 10 degree
				if(degree>=18){
					state=1;//go down state
					degree=18;// degree is 180 degree
				}
		}
			else{//down state
			degree--;//minus 10 degree
			if(degree<=0){
				state=0;//go up state
				degree=0;//degree is 0 degree
			}
		}
			pulse_width=0.5+degree/18.0*(2.5-0.5);//calculate pulse width
			PWM_pulsewidth(Moter_pin,pulse_width);//chagne pulse width
			_count=0;//reset
	}
		
		clear_UIF(TIM3); 		// Clear UI flag by writing 0
	}
}

int count=0;
//button intterupt
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(PC_13)) {//check PC_13 is pending
		pulse_width=0.5;//0 degree pulse width is 0.5msec
		state=0;//go up state
		degree=0;//degree is 0
		PWM_pulsewidth(Moter_pin,pulse_width);//change pulse width
		clear_pending_EXTI(PC_13); // cleared by writing '1'
	}
}
