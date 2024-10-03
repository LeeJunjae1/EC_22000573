//DC MOTOR

#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"
#include "math.h"


#define Moter_pin PA_0    
float duty=0.25;//pulse width
void setup(void);
void LED_toggle() {
    GPIOA->ODR^=1<<5;
}

int main(void) {
	// Initialization --------------------------------------------------
	setup();
		//PWM
	/*
	RCC_PLL_init();				// System Clock = 84MHz
	PWM_init(Moter_pin);
	PWM_duty(Moter_pin,1);//change duty
*/
	
	// TEMP: TIMER Register Initialiization --------------------------------------------------------		



	while(1){}
}


// Initialization
void setup(void){
	

	//TIM IN tu
	RCC_PLL_init();				// System Clock = 84MHz
	//GPIO_init(LED_pin, OUTPUT);	// calls RCC_GPIOA_enable()
	GPIO_init(PC_13, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PC_13, EC_UP);//pull-up
	
	//Direct pin
	GPIO_init(PC_2, OUTPUT); //Calls RCC_GPIO_enable(), output
	
	//EXTI INIT
	EXTI_init(PC_13, FALL, 0);//falling edge, priority=0
	
	//100 = 1msec
	PWM_init(Moter_pin);
	//PWM_duty(Moter_pin,1);//change duty
	PWM_period(Moter_pin,100);
	PWM_duty(LED_pin,duty);//change duty

	
	//100 = 1msec
	TIM_UI_init(TIM3, 100);			// TIM2 Update-Event Interrupt every 1 msec 
	TIM_UI_enable(TIM3);
	
	/*
	//TIM IN tu
	RCC_PLL_init();				// System Clock = 84MHz
	GPIO_init(LED_pin, OUTPUT);	// calls RCC_GPIOA_enable()
	//100 = 1msec
	//PWM_init(LED_pin);
	
	TIM_UI_init(TIM2, 100);			// TIM2 Update-Event Interrupt every 1 msec 
	TIM_UI_enable(TIM2);*/
	
	

}
int state=0;//0->up, 1->down
uint32_t _count = 0;
uint32_t degree = 0;
//14:22 gunzogi
void TIM3_IRQHandler(void){
	float change_duty=duty;
	duty=fabs(1.0-change_duty);//change duty
	if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
		_count++;
		if(_count>500){
	PWM_duty(Moter_pin,duty);//change duty
			_count=0;
	}
		
		clear_UIF(TIM3); 		// Clear UI flag by writing 0
	}
}

/*
void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){			// Check UIF(update interrupt flag)
		_count++;
		if (_count > 1000) {
			LED_toggle();		// LED toggle every 1 sec
			_count = 0;
		}
		clear_UIF(TIM2); 		// Clear UI flag by writing 0
	}
}*/
int count=0;
//button intterupt
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(PC_13)) {//check PC_13 is pending
		TIM2->CR1^=1;//TOGGLE enable clock
		clear_pending_EXTI(PC_13); // cleared by writing '1'
	}
}



/*
#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"


#define Moter_pin PA_5    
float pulse_width=0.5;//pulse width
void setup(void);
void LED_toggle() {
    GPIOA->ODR^=1<<5;
}

int main(void) {
	// Initialization --------------------------------------------------
	setup();
		//PWM*/
	/*
	RCC_PLL_init();				// System Clock = 84MHz
	PWM_init(Moter_pin);
	PWM_duty(Moter_pin,1);//change duty
*/
	
	// TEMP: TIMER Register Initialiization --------------------------------------------------------		



//	while(1){}
//}

/*
// Initialization
void setup(void){
	

	//TIM IN tu
	RCC_PLL_init();				// System Clock = 84MHz
	//GPIO_init(LED_pin, OUTPUT);	// calls RCC_GPIOA_enable()
	GPIO_init(PC_13, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PC_13, EC_UP);//pull-up
	
	//EXTI INIT
	EXTI_init(PC_13, FALL, 0);//falling edge, priority=0
	
	//100 = 1msec
	PWM_init(Moter_pin);
	//PWM_duty(Moter_pin,1);//change duty
	PWM_period(Moter_pin,2000);
	//PWM_duty(LED_pin,pulse_width);//change duty
	PWM_pulsewidth(Moter_pin,pulse_width);
	
	//100 = 1msec
	TIM_UI_init(TIM3, 100);			// TIM2 Update-Event Interrupt every 1 msec 
	TIM_UI_enable(TIM3);*/
	
	/*
	//TIM IN tu
	RCC_PLL_init();				// System Clock = 84MHz
	GPIO_init(LED_pin, OUTPUT);	// calls RCC_GPIOA_enable()
	//100 = 1msec
	//PWM_init(LED_pin);
	
	TIM_UI_init(TIM2, 100);			// TIM2 Update-Event Interrupt every 1 msec 
	TIM_UI_enable(TIM2);*/
	
	/*

}
int state=0;//0->up, 1->down
uint32_t _count = 0;
uint32_t degree = 0;
//14:22 gunzogi
void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
		_count++;
		if(_count>500){
			if(state==0){
				degree++;
				if(degree>=18){
					state=1;
					degree=18;
				}
		}
			else{
			degree--;
			if(degree<=0){
				state=0;
				degree=0;
			}
		}
			pulse_width=0.5+degree/18.0*(2.5-0.5);
			PWM_pulsewidth(Moter_pin,pulse_width);
			_count=0;
	}
		
		clear_UIF(TIM3); 		// Clear UI flag by writing 0
	}
}*/

/*
void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){			// Check UIF(update interrupt flag)
		_count++;
		if (_count > 1000) {
			LED_toggle();		// LED toggle every 1 sec
			_count = 0;
		}
		clear_UIF(TIM2); 		// Clear UI flag by writing 0
	}
}*/

/*
int count=0;
//button intterupt
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(PC_13)) {//check PC_13 is pending
		pulse_width=0.5;
		state=0;
		degree=0;
		//sevensegment_display(count);//display number
		PWM_pulsewidth(Moter_pin,pulse_width);
		clear_pending_EXTI(PC_13); // cleared by writing '1'
	}
}*/