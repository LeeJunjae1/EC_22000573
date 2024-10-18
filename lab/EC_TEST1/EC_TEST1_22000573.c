/**
******************************************************************************
* @author	Junjae Lee
* @Mod		10-15-2024
* @brief	Embedded Controller:  EC TEST1
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "math.h"

/*
int count = 0;//state num
int count_2 = 0;//display num
int count_3 = 0;//second num
int pause = 0;
int IR_com = 0;
int IR_go=0;
int dir = 0; //direction work

#define PWM_PIN PA_0

//int count = 0;
// Initialiization 
void setup(void)
{
	RCC_PLL_init();//PLL INIT

	//button pin
	GPIO_init(PC_13, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PC_13, EC_DOWN);//pull-down
	
	//7 segment display
	sevensegment_display_init(PA_7,PB_6,PC_7,PA_9);//use PA_7:D, PB_6:C, PC_7:B, PA_9:A

	
	//button EXTI INIT
	EXTI_init(PC_13, RISE, 0);//falling edge, priority=0
	NVIC_EnableIRQ(EXTI15_10_IRQn); // enable request interrupt
	NVIC_SetPriority(EXTI15_10_IRQn, 3); // set priority
	
	//ir proximity sensor
	GPIO_init(PB_1, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PB_1, EC_DOWN);//pull-down
	
	//ir exti
	EXTI_init(PB_1, RISE, 0);//falling edge, priority=0


	//direction setting
	GPIO_init(PC_2,OUTPUT);
	GPIO_otype(PC_2, 0);//push pull
	GPIO_write(PC_2,0);//low
	
	//pwm
	PWM_init(PA_0);
	
	//TIM2
	TIM_init(TIM2); //1msec
	TIM_UI_enable(TIM2);
	NVIC_EnableIRQ(TIM2_IRQn);	// TIM2's interrupt request enabled	
	NVIC_SetPriority(TIM2_IRQn, 3);
	
	//TIM3
	//500 msec
	TIM_UI_init(TIM3,500);
	TIM_UI_enable(TIM3);
	NVIC_EnableIRQ(TIM3_IRQn);	// TIM3's interrupt request enabled	
	NVIC_SetPriority(TIM3_IRQn, 2);
	
}


int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
	}
}

void TIM3_IRQHandler(void){
		if(is_UIF(TIM3)){ 
			count_3++;
			if(count == 0) {
				//7 segment off
				GPIO_write(PA_7,1);
				GPIO_write(PB_6,1);
				GPIO_write(PC_7,1);
				GPIO_write(PA_9,1);
				count_3=0;
				count_2=0;
		
			}
			else if(count == 1) {// press 1
				
				if(count_3 >= 2) {
					if(dir==0){//cw
					
					sevensegment_display(count_2);//display number
					count_2++;//up counting
					if(count_2>9){
						count_2=0;
					}
				}
					else{//ccw
						if(count_2==0){
							count_2=10;
						}
					count_2--;//down counting
					sevensegment_display(count_2);//display number
					if(count_2<1){
						count_2=10;
					}
					}
					count_3 = 0;
			 }
		 }
		 else if(count ==2){//press 2
			 if(dir==0){//cw
			
			sevensegment_display(count_2);//display number
				count_2++;//up counting
			 if(count_2>9){
						count_2=0;
					}
				}
			 else{//ccw
				 if(count_2==0){
							count_2=10;
						}
				 count_2--;//down counting
				 sevensegment_display(count_2);//display number
					if(count_2<1){
						count_2=10;
					}
			 }
			 count_3=0;
				
			}
		}
		// clear by writing 0
		clear_UIF(TIM3);                 
 
	
	
}

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){ 
		float change_duty;
		change_duty=fabs(dir-(0.35*count));//change duty

		PWM_duty(PWM_PIN, change_duty);
		
		// clear by writing 0
		clear_UIF(TIM2); 		// Clear UI flag by writing 0              
	}
}

void EXTI15_10_IRQHandler(void) {
	if ((is_pending_EXTI(PC_13)&&IR_go==1&&IR_com==0)||(is_pending_EXTI(PC_13)&&IR_go==1&&IR_com==0)||(is_pending_EXTI(PC_13)&&IR_go==0&&IR_com==0)) {//check PC_13 is pending
		for(volatile int i = 0; i < 1000000; i++){}
		  count++;
			if(count > 2) 
			{
				count = 0;	//reset count
				dir =! dir;//toggle direction
				GPIO_write(PC_2, dir);
			}
		clear_pending_EXTI(PC_13); // cleared by writing '1'
	}
}

void EXTI1_IRQHandler(void) {
	if (is_pending_EXTI(PB_1)) {//check PB_1 is pending
		for(volatile int i = 0; i < 1000000; i++){}
			float change_duty;
			if((IR_go==0&&IR_com==0)||(IR_go==1&&IR_com==0)){
				IR_com=1;
				IR_go=0;
				change_duty=fabs(dir-0.0);//change duty
				PWM_duty(PWM_PIN, change_duty);
				if (dir==1){
					GPIO_write(PC_2, 0);
					PWM_duty(PWM_PIN, 0);
				}
				dir=0;
			}
			else if(IR_com==1&&IR_go==0){
				IR_go=1;
				IR_com=0;
				change_duty=fabs(dir-0.0);//change duty
				PWM_duty(PWM_PIN, change_duty);
					if (dir==1){
					GPIO_write(PC_2, 0);
					PWM_duty(PWM_PIN, 0);
				}
				dir=0;
			}
			count=0;
			GPIO_write(PA_7,1);
			GPIO_write(PB_6,1);
			GPIO_write(PC_7,1);
			GPIO_write(PA_9,1);
			count_2=0;
			count_3=0;
			//float change_duty=0.0;
			
			//change_duty=fabs(dir-1.0);//change duty
			//PWM_duty(PWM_PIN, change_duty);
			dir=0;
			
			
		clear_pending_EXTI(PB_1); // cleared by writing '1'
	}
}*/

int count = 0;//state num
int count_2 = 0;//display num
int count_3 = 0;//second num
int pause = 0;
int IR_com = 0;
int IR_go=0;
int dir = 0; //direction work

#define PWM_PIN PA_0

volatile uint32_t pressed_flag=0;

float moterPWMState[6]={0,0.4f,0.7f, 0,0.4f,0.7f};
uint32_t moterOnState[6]={0,1,1, 0,1,1};
uint32_t moterDIRState[6]={0,0,0,0,1,1};
uint32_t blinkRateState[6]={0,1,0, 0,1,0};

volatile uint32_t currentState=0;
volatile float moterPWM=0;
volatile uint32_t moterOn=0;
volatile uint32_t moterDIR=0;
volatile uint32_t blinkRate=0;


volatile uint32_t _cntTimer=0;
volatile uint32_t _cntNum=0;

volatile uint32_t IR_flag=0;




void updateState(){
	uint32_t k=currentState;
	moterPWM=moterPWMState[k];
	moterOn=moterOnState[k];
	moterDIR=moterDIRState[k];
	blinkRate=blinkRateState[k];
	
	currentState++;
	if(currentState>5)currentState=0;
}

//int count = 0;
// Initialiization 
void setup(void)
{
	RCC_PLL_init();//PLL INIT

	//button pin
	GPIO_init(PC_13, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PC_13, EC_DOWN);//pull-down
	
	//7 segment display
	sevensegment_display_init(PA_7,PB_6,PC_7,PA_9);//use PA_7:D, PB_6:C, PC_7:B, PA_9:A

	
	//button EXTI INIT
	EXTI_init(PC_13, RISE, 0);//falling edge, priority=0
	NVIC_EnableIRQ(EXTI15_10_IRQn); // enable request interrupt
	NVIC_SetPriority(EXTI15_10_IRQn, 3); // set priority
	
	//ir proximity sensor
	GPIO_init(PB_1, INPUT); //Calls RCC_GPIO_enable(), INPUT
	GPIO_pupd(PB_1, EC_DOWN);//pull-down
	
	//ir exti
	EXTI_init(PB_1, RISE, 0);//falling edge, priority=0


	//direction setting
	GPIO_init(PC_2,OUTPUT);
	GPIO_otype(PC_2, 0);//push pull
	GPIO_write(PC_2,0);//low
	
	//pwm
	PWM_init(PA_0);
	
	//TIM2
	TIM_init(TIM2); //1msec
	TIM_UI_enable(TIM2);
	NVIC_EnableIRQ(TIM2_IRQn);	// TIM2's interrupt request enabled	
	NVIC_SetPriority(TIM2_IRQn, 3);
	
	//TIM3
	//500 msec
	TIM_UI_init(TIM3,500);
	TIM_UI_enable(TIM3);
	NVIC_EnableIRQ(TIM3_IRQn);	// TIM3's interrupt request enabled	
	NVIC_SetPriority(TIM3_IRQn, 2);
	
}


int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
	}
}

void TIM3_IRQHandler(void){
		if(is_UIF(TIM3)){ 
			//far away and button pressed
			//if default=H
			IR_flag=GPIO_read(PB_1);
			if(pressed_flag&IR_flag){
				updateState();
				pressed_flag=0;
			}
			
			_cntTimer++;
			
			if(_cntTimer>blinkRate){
				_cntTimer=0;
				if(!moterOn){
				GPIO_write(PA_7,1);
				GPIO_write(PB_6,1);
				GPIO_write(PC_7,1);
				GPIO_write(PA_9,1);
			}
			else{
				sevensegment_display(_cntNum%10);
				if(!moterDIR){
					_cntNum++;
					if(_cntNum>9)_cntNum=0;
				}
				else{
					_cntNum--;
					if(_cntNum<0)_cntNum=9;
				}
				
			}
				
			}
			
			
		}
		// clear by writing 0
		clear_UIF(TIM3);                 
 
	
	
}

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){ 
		//float change_duty;
		//change_duty=fabs(dir-(0.35*count));//change duty

		//PWM_duty(PWM_PIN, change_duty);
		
		float duty=fabs(moterDIR-moterPWM);
		PWM_duty(PB_1,duty);
		
		// clear by writing 0
		clear_UIF(TIM2); 		// Clear UI flag by writing 0              
	}
}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(PC_13)) {//check PC_13 is pending
			pressed_flag=1;
		clear_pending_EXTI(PC_13); // cleared by writing '1'
	}
}

void EXTI1_IRQHandler(void) {
	if (is_pending_EXTI(PB_1)) {//check PB_1 is pending
		IR_flag=GPIO_read(PB_1);//proximity:0 default:1
		
		if(!IR_flag){
			currentState=0;
			updateState();
			
			float duty=fabs(moterDIR-moterPWM);
			PWM_duty(PB_1,duty);
			
		}

		clear_pending_EXTI(PB_1); // cleared by writing '1'
	}
}
