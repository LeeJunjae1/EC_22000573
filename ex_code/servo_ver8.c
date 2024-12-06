/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-11-15 by JunjaeLee
* @brief   Embedded Controller: Final Project RC car
*
******************************************************************************
*/

#define _CRT_SECURE_NO_WARNINGS  
#include <stdio.h>
#include "stm32f411xe.h"
#include "math.h"
#include "stdlib.h"
#include "ecPinNames.h"
#include "ecRCC2.h"
#include "ecGPIO2.h"
#include "ecEXTI2.h"
#include "ecSysTick2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "ecICAP2.h"
#include "ecUART2.h"
#include "ecADC2.h"

#define END_CHAR 13
#define MAX_BUF 100

uint8_t PC_Data = 0;//pcdata
uint8_t BT_Data = 0;//bluetooth data
uint8_t buffer[MAX_BUF] = {0, };


#define Servo_pin PC_6 //Left back servo motor
#define Servo_pin1 PC_7    //Right front servo motor
#define Servo_pin2 PC_8 //Right back servo motor
#define Servo_pin3 PC_9    //Left front servo motor

#define Motor_1 PA_1 //TIM2 Ch1 Right front DC motor
#define Motor_2 PA_0 //TIM2 Ch2 Left front DC motor
#define Motor_3 PA_5 //TIM2 Ch3 Left back DC motor
#define Motor_4 PB_3 //TIM2 Ch4 Right back DC motor

#define Dir_Pin_1 PC_3 //direction pin for right front
#define Dir_Pin_2 PC_2 //direction pin for left front
#define Dir_Pin_3 PC_13 //direction pin for left back
#define Dir_Pin_4 PB_7 //direction pin for right back

#define Ultra_pin1 PB_2 //ultra sensor1
#define Ultra_pin2 PB_4 //ultra sensor2
#define Parking_pin PB_5 //parking flag

unsigned int pulse_width=2500;//Left back servo motor pulse width
unsigned int pulse_width1=2500;//Right front servo motor pulse width
unsigned int pulse_width2=500;//Right back servo motor pulse width
unsigned int pulse_width3=600;//Left front servo motor pulse width

//IR parameter//
uint32_t value1, value2, value3;//value1: right ir sensor, value2: left ir sensor, value3: Infrared sensor
PinName_t seqCHn[3] = {PB_0, PB_1, PC_4}; //use to pin as jadc

volatile uint32_t motorDIR; //motor1 direction
volatile float motorPWM=0.6; //motor1 PWM duty

volatile uint32_t motorDIR_2; //motor2 direction
volatile float motorPWM_2=0.6; //motor2 PWM duty

volatile uint32_t motorDIR_3; //motor3 direction
volatile float motorPWM_3=0.6; //motor3 PWM duty

volatile uint32_t motorDIR_4; //motor4 direction
volatile float motorPWM_4=0.6; //motor4 PWM duty

//motor duty
float duty=1.0;//motor1 duty
float duty_2=1.0;//motor2 duty
float duty_3=1.0;//motor3 duty
float duty_4=1.0;//motor4 duty

volatile uint32_t Ultra_flag=0;// to check front object
volatile uint32_t Ultra_flag2=0;// to check back object

volatile uint32_t Mode_flag=0;// check mode is auto or manual
volatile uint32_t Change_flag=0;// to check rotating car
volatile uint32_t Parking_flag=0;// to check parking place
volatile uint32_t Parking_check_flag=0;// to check parking flag

volatile uint32_t Back_flag=0;// to check moving backward when parking mode
volatile uint32_t finish_flag=0;// finish parking
volatile uint32_t fire_flag=0;// to check fire
volatile uint32_t Go_fire_flag=0;//go flag when detecting fire
volatile uint32_t Turn_flag=0;// rotate rc car(fire ditection)
volatile uint32_t Finish_turn_flag=0;//end of RC car ratation(fire detection)

uint32_t cnt=0;//tim4 interuppt 
uint32_t cnt1=0;//to read sensor value

//for change duty
float motorPWMState[4]={0.5f,0.65f,0.85f, 1.0f};// Straight speed
uint8_t PWMstate=0;//motor speed state

int STRval=0; //STR value, negative: left, positive: right
int STRabs=0;//for str value abs

void setup(void){
	RCC_PLL_init();
	SysTick_init(1000);
	
	// USART2: USB serial init
	UART2_init();
	UART2_baud(BAUD_9600);

	// USART1: BT serial init 
	UART1_init();
	UART1_baud(BAUD_9600);
	
	GPIO_init(Ultra_pin1, INPUT);  //read ultra1(front)
	GPIO_init(Ultra_pin2, INPUT);  //read ultra2(back)
	GPIO_init(Parking_pin, INPUT);  //read parking place
	
	//pull down
	GPIO_pupd(Ultra_pin1,2);
	GPIO_pupd(Ultra_pin2,2);
	GPIO_pupd(Parking_pin,2);
	
	//motor1(right front DC motor)
	PWM_init(Motor_1);// pwm settnig
	PWM_period_us(Motor_1,200);//0.2msec, 5kHz
	PWM_duty(Motor_1,duty);//change duty
	
	//motor2(left front DC motor)
	PWM_init(Motor_2);// pwm settnig
	PWM_period_us(Motor_2,200);//0.2msec, 5kHz
	PWM_duty(Motor_2,duty_2);//change duty
	
	//motor3(left back DC motor)
	PWM_init(Motor_3);// pwm settnig
	PWM_period_us(Motor_3,200);//0.2msec, 5kHz
	PWM_duty(Motor_3,duty_3);//change duty
	
	//motor3(right back DC motor)
	PWM_init(Motor_4);// pwm settnig
	PWM_period_us(Motor_4,200);//0.2msec, 5kHz
	PWM_duty(Motor_4,duty_4);//change duty
	
	
	//direction
	GPIO_init(Dir_Pin_1, OUTPUT);  //direction1(right front direction)
	GPIO_init(Dir_Pin_2, OUTPUT);  //direction2(left front direction)
	GPIO_otype(Dir_Pin_1, 0);//push-pull
	GPIO_otype(Dir_Pin_2, 0);//push-pull
	GPIO_write(Dir_Pin_1,1);//moving forward
	GPIO_write(Dir_Pin_2,1);//moving forward
	
	GPIO_init(Dir_Pin_3, OUTPUT);  //direction3(left back direction)
	GPIO_init(Dir_Pin_4, OUTPUT);  //direction4(right back direction)
	GPIO_otype(Dir_Pin_3, 0);//push-pull
	GPIO_otype(Dir_Pin_4, 0);//push-pull
	GPIO_write(Dir_Pin_3,1);//moving forward
	GPIO_write(Dir_Pin_4,1);//moving forward
	
//read motor direction
	motorDIR=GPIO_read(Dir_Pin_1);
	motorDIR_2=GPIO_read(Dir_Pin_2);
	motorDIR_3 =GPIO_read(Dir_Pin_3);
	motorDIR_4=GPIO_read(Dir_Pin_4);
	
	//TIM2
	TIM_UI_enable(TIM2);//TIMER2 enable
	NVIC_EnableIRQ(TIM2_IRQn);	// TIM2's interrupt request enabled	
	//NVIC_SetPriority(TIM2_IRQn,3);
	
	//servo(Left back servo motor)
	PWM_init(Servo_pin);//initialize pwm
	
	GPIO_pupd(Servo_pin,1);//pull-up
	GPIO_ospeed(Servo_pin,Fast_speed);//fast
	
	PWM_period_us(Servo_pin,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin,pulse_width);//to change pulse width
	
	//servo1(Right front servo motor)
	PWM_init(Servo_pin1);//initialize pwm

	GPIO_pupd(Servo_pin1,1);//pull-up
	GPIO_ospeed(Servo_pin1,Fast_speed);//fast
	
	PWM_period_us(Servo_pin1,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin1,pulse_width1);//to change pulse width
	
	//servo2(Right back servo motor)
	PWM_init(Servo_pin2);//initialize pwm
	
	GPIO_pupd(Servo_pin2,1);//pull-up
	GPIO_ospeed(Servo_pin2,Fast_speed);//fast
	
	PWM_period_us(Servo_pin2,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin2,pulse_width2);//to change pulse width
	
	
	//servo3(Left front servo motor)
	PWM_init(Servo_pin3);//initialize pwm
	
	GPIO_pupd(Servo_pin3,1);//pull-up
	GPIO_ospeed(Servo_pin3,Fast_speed);//fast
	
	PWM_period_us(Servo_pin3,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin3,pulse_width3);//to change pulse width


// JADC Init
	JADC_init(PB_0);
	JADC_init(PB_1);
	JADC_init(PC_4);

	// ADC channel sequence setting
	JADC_sequence(seqCHn, 3);
	
	//TIM5 enable for ADC
	TIM_UI_enable(TIM5);
	NVIC_EnableIRQ(TIM5_IRQn);
	NVIC_SetPriority(TIM5_IRQn, 1);
}

int main(void){	
	setup();
	printf("Hello Nucleo\r\n");
	USART_write(USART1,(uint8_t*) "Hello bluetooth\r\n",17);
	while(1){       
	}
}


void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART_RXNE(USART1)){
		BT_Data = USART_read(USART1);		// RX from UART1 (BT)		
		//check direction key
		if (BT_Data == 0x1B) { // ESC key
            uint8_t nextChar1 = USART_read(USART1);
            if (nextChar1 == '[') {
                uint8_t nextChar2 = USART_read(USART1);
                switch (nextChar2) {
                    case 'A': // up key
											PWMstate++;//Speed up
										if(PWMstate>3){
											PWMstate=3;
										}
										//change motor pwm
										motorPWM=motorPWMState[PWMstate];
										motorPWM_2=motorPWMState[PWMstate];
										motorPWM_3=motorPWMState[PWMstate];
										motorPWM_4=motorPWMState[PWMstate];
										
                        break;
										
                    case 'B': // down key
										if(PWMstate==0){
											PWMstate=1;
										}
										//speed down
										PWMstate--;
										//change motor pwm
										motorPWM=motorPWMState[PWMstate];
										motorPWM_2=motorPWMState[PWMstate];
										motorPWM_3=motorPWMState[PWMstate];
										motorPWM_4=motorPWMState[PWMstate];
                        break;
										
                    case 'C': // right key
											STRval++;
										if(STRval>3){
											STRval=3; //MAX 3
										}
										STRabs=abs(STRval);
										if(STRval>0){
											//Turn right
										motorPWM=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//right front motor speed
										motorPWM_2=motorPWMState[PWMstate];//left front motor speed
										motorPWM_3=motorPWMState[PWMstate];//left back motor speed
										motorPWM_4=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;// right back motor speed
										
										}
										else if(STRval==0){
											//Go straight
											motorPWM=motorPWMState[PWMstate];//right front motor speed
											motorPWM_2=motorPWMState[PWMstate];//left front motor speed
											motorPWM_3=motorPWMState[PWMstate];//left back motor speed
											motorPWM_4=motorPWMState[PWMstate];//right back motor speed

										}
										else if(STRval<0){
											//turn left
											motorPWM=motorPWMState[PWMstate];//right front motor speed
											motorPWM_2=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left front motor speed
											motorPWM_3=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left back motor speed
											motorPWM_4=motorPWMState[PWMstate];//right back motor speed
										}
                        break;
										
                    case 'D': // left key
										STRval--;
										if(STRval<-3){
											STRval=-3;//MIN -3
										}
										STRabs=abs(STRval);
										if(STRval>0){
											//Turn right
										motorPWM=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//right front motor speed
										motorPWM_2=motorPWMState[PWMstate];//left front motor speed
										motorPWM_3=motorPWMState[PWMstate];//left back motor speed
										motorPWM_4=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//right back motor speed
										
										}
										else if(STRval==0){
											//Go straight
											motorPWM=motorPWMState[PWMstate];//right front motor speed
											motorPWM_2=motorPWMState[PWMstate];//left front motor speed
											motorPWM_3=motorPWMState[PWMstate];//left back motor speed
											motorPWM_4=motorPWMState[PWMstate];//right back motor speed
										
										}
										else if(STRval<0){
											//turn left
											motorPWM=motorPWMState[PWMstate];//right front motor speed
											motorPWM_2=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left front motor speed
											motorPWM_3=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left back motor speed
											motorPWM_4=motorPWMState[PWMstate];//right back motor speed

										}
										
                        break;
                    default:
                        break;
									}
								
            }
        } 
				else{
			if((BT_Data=='M')||(BT_Data=='m')){
					Change_flag=0;//no rotation
					Mode_flag=0;//manual mode
					STRval=0;//str value initialize
					PWMstate=0; //velocity state initialize
					fire_flag=0;//no fire
					Ultra_flag=0;//no object(front)
					Ultra_flag2=0;//no object(back)
					Parking_flag=0;//no parkiing place
					Back_flag=0;//no moving backward when pakring mode
				}
				else if((BT_Data=='N')||(BT_Data=='n')){
					Mode_flag=1;//auto mode
					Change_flag=0;//no rotation
					Ultra_flag=0;//no object(front)
					Ultra_flag2=0;//no object(back)
					Parking_flag=0;//no parking place
					finish_flag=0;//no finish parking
				}
				
				//moving forward
				else if(((BT_Data=='W')||(BT_Data=='w'))&&(Mode_flag==0)&&(Change_flag==0)){
					//0 degree
					pulse_width=2500;
					pulse_width1=2500;
					pulse_width2=500;
					pulse_width3=600;
					
					//Go straight
					GPIO_write(Dir_Pin_1,1);
					GPIO_write(Dir_Pin_2,1);
					GPIO_write(Dir_Pin_3,1);
					GPIO_write(Dir_Pin_4,1);
					
					//read motor dir
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);
					
					//change motor duty
					motorPWM=motorPWMState[PWMstate];
					motorPWM_2=motorPWMState[PWMstate];
					motorPWM_3=motorPWMState[PWMstate];
					motorPWM_4=motorPWMState[PWMstate];
				}
				
				//moving backward
				else if(((BT_Data=='S')||(BT_Data=='s'))&&(Mode_flag==0)&&(Change_flag==0)){
					//0 degree
					pulse_width=2500;
					pulse_width1=2500;
					pulse_width2=500;
					pulse_width3=600;
					
					//Back
					GPIO_write(Dir_Pin_1,0);//change direction
					GPIO_write(Dir_Pin_2,0);//change direction
					//read motor direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					
					GPIO_write(Dir_Pin_3,0);//change direction
					GPIO_write(Dir_Pin_4,0);//change direction
					//read motor direction
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);
					
					//change motor duty
					motorPWM=motorPWMState[PWMstate];
					motorPWM_2=motorPWMState[PWMstate];
					motorPWM_3=motorPWMState[PWMstate];
					motorPWM_4=motorPWMState[PWMstate];
				}
				else if(((BT_Data=='a')||(BT_Data=='A'))&&(Mode_flag==0)&&(Change_flag==0)){
					//turn left
					//change motor duty
					motorPWM=1.0;//right front DC motor
					motorPWM_2=0.3;//left front DC motor
					motorPWM_3=0.3;//left back DC motor
					motorPWM_4=1.0;//righ back DC motor
				}
				else if(((BT_Data=='D')||(BT_Data=='d'))&&(Mode_flag==0)&&(Change_flag==0)){
					//turn right
					//change motor duty
					motorPWM=0.3;//right front DC motor
					motorPWM_2=1.0;//left front DC motor
					motorPWM_3=1.0;//left back DC motor
					motorPWM_4=0.3;//right back DC motor
				}
				//stop RC car
				else if(((BT_Data=='F')||(BT_Data=='f'))&&(Mode_flag==0)){
					//read motor direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);
					
					//stop rc car
					switch(motorDIR){
					case 0:
						motorPWM=motorDIR;
					break;
					case 1:
						motorPWM=!motorDIR;
				}
				
				switch(motorDIR_2){
					case 0:
						motorPWM_2=motorDIR_2;
					break;
					case 1:
						motorPWM_2=!motorDIR_2;
				}
				switch(motorDIR_3){
					case 0:
						motorPWM_3=motorDIR_3;
					break;
					case 1:
						motorPWM_3=!motorDIR_3;
				}
				switch(motorDIR_4){
					case 0:
						motorPWM_4=motorDIR_4;
					break;
					case 1:
						motorPWM_4=!motorDIR_4;
				}
					
					
					//STR, VEL initialize
					STRval=0;
					PWMstate=0;
				}
				
				
				//45degree
				else if(BT_Data=='L'){
					//change pulse width of servo motor
				pulse_width=2000;
				pulse_width1=2000;
				pulse_width2=1000;
				pulse_width3=1000;
					
				//stop motor
				motorDIR=GPIO_read(Dir_Pin_1);
				motorDIR_2=GPIO_read(Dir_Pin_2);
				motorDIR_3=GPIO_read(Dir_Pin_3);
				motorDIR_4=GPIO_read(Dir_Pin_4);
					
					//stop RC car
					switch(motorDIR){
					case 0:
						motorPWM=motorDIR;
					break;
					case 1:
						motorPWM=!motorDIR;
				}
				
				switch(motorDIR_2){
					case 0:
						motorPWM_2=motorDIR_2;
					break;
					case 1:
						motorPWM_2=!motorDIR_2;
				}
				switch(motorDIR_3){
					case 0:
						motorPWM_3=motorDIR_3;
					break;
					case 1:
						motorPWM_3=!motorDIR_3;
				}
				switch(motorDIR_4){
					case 0:
						motorPWM_4=motorDIR_4;
					break;
					case 1:
						motorPWM_4=!motorDIR_4;
				}
			Change_flag=1;//chagne servo
			
		}
				//90 degree
		else if(BT_Data=='H'){
			//change pulse width of servo motor
			pulse_width=1500;
			pulse_width1=1700;
			pulse_width2=1300;
			pulse_width3=1400;
			
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);	
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorDIR_3=GPIO_read(Dir_Pin_3);	
			motorDIR_4=GPIO_read(Dir_Pin_4);
			
			//stop RC car
			switch(motorDIR){
					case 0:
						motorPWM=motorDIR;
					break;
					case 1:
						motorPWM=!motorDIR;
				}
				
				switch(motorDIR_2){
					case 0:
						motorPWM_2=motorDIR_2;
					break;
					case 1:
						motorPWM_2=!motorDIR_2;
				}
				switch(motorDIR_3){
					case 0:
						motorPWM_3=motorDIR_3;
					break;
					case 1:
						motorPWM_3=!motorDIR_3;
				}
				switch(motorDIR_4){
					case 0:
						motorPWM_4=motorDIR_4;
					break;
					case 1:
						motorPWM_4=!motorDIR_4;
				}
			Change_flag=1;// change servo
		}
		
		//0 degree
		else if(BT_Data=='K'){
			//change puslewidth of servo motor
			pulse_width=2500;
			pulse_width1=2500;
			pulse_width2=500;
			pulse_width3=600;
			
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);		
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorDIR_3=GPIO_read(Dir_Pin_3);
			motorDIR_4=GPIO_read(Dir_Pin_4);
			
			//stop RC car
			switch(motorDIR){
					case 0:
						motorPWM=motorDIR;
					break;
					case 1:
						motorPWM=!motorDIR;
				}
				
				switch(motorDIR_2){
					case 0:
						motorPWM_2=motorDIR_2;
					break;
					case 1:
						motorPWM_2=!motorDIR_2;
				}
				switch(motorDIR_3){
					case 0:
						motorPWM_3=motorDIR_3;
					break;
					case 1:
						motorPWM_3=!motorDIR_3;
				}
				switch(motorDIR_4){
					case 0:
						motorPWM_4=motorDIR_4;
					break;
					case 1:
						motorPWM_4=!motorDIR_4;
				}
			
			
		
			Change_flag=0;//no rotation
		}
		
		//Rotation
		else if(BT_Data=='I'&&(Change_flag==1)){

			//change direction to rotation
			GPIO_write(Dir_Pin_1,1);
			GPIO_write(Dir_Pin_2,0);
			GPIO_write(Dir_Pin_3,0);
			GPIO_write(Dir_Pin_4,1);

			//change duty ratio of DC motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=0.8;//rotation
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=0.8;//rotation
			
			motorDIR_3=GPIO_read(Dir_Pin_3);
			motorPWM_3=0.8;//rotation
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
			motorPWM_4=0.8;//rotation

			Change_flag=1;//RC car rotation
		}
		
		else if(BT_Data=='O'){

			//change direction to rotation
			GPIO_write(Dir_Pin_1,1);//Right front
			GPIO_write(Dir_Pin_2,0);//Left front
			GPIO_write(Dir_Pin_3,1);//Left back
			GPIO_write(Dir_Pin_4,0);//Right back

			//change duty ratio of DC motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=1.0;//rotation
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=1.0;//rotation
			
			motorDIR_3=GPIO_read(Dir_Pin_3);
			motorPWM_3=1.0;//rotation
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
			motorPWM_4=1.0;//rotation

//			Change_flag=1;
		}
		
		else if(BT_Data=='P'){

			//change direction to rotation
			GPIO_write(Dir_Pin_1,0);//Right front
			GPIO_write(Dir_Pin_2,1);//Left front
			GPIO_write(Dir_Pin_3,0);//Left back
			GPIO_write(Dir_Pin_4,1);//Right back

			//change duty ratio of DC motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=1.0;//rotation
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=1.0;//rotation
			
			motorDIR_3=GPIO_read(Dir_Pin_3);
			motorPWM_3=1.0;//rotation
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
			motorPWM_4=1.0;//rotation

	//		Change_flag=1;
		}
		
		
		}
	}
}

uint32_t _count = 0;//for parking
uint32_t _count1 = 0;//for escape parking place when detecting fire


void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){

		//auto mode
		if(Mode_flag==1){
			//find parking place and no moving backward and no finsh parking
			if((Parking_flag==1)&&(Back_flag==0)&&(finish_flag==0)){
				//PARKING START
				
				//STOP MOTOR
				//read direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);
				
				//stop RC car
				switch(motorDIR){
					case 0:
						motorPWM=motorDIR;
					break;
					case 1:
						motorPWM=!motorDIR;
				}
				
				switch(motorDIR_2){
					case 0:
						motorPWM_2=motorDIR_2;
					break;
					case 1:
						motorPWM_2=!motorDIR_2;
				}
				switch(motorDIR_3){
					case 0:
						motorPWM_3=motorDIR_3;
					break;
					case 1:
						motorPWM_3=!motorDIR_3;
				}
				switch(motorDIR_4){
					case 0:
						motorPWM_4=motorDIR_4;
					break;
					case 1:
						motorPWM_4=!motorDIR_4;
				}
				
					_count++;
					if(_count<20000){
						//to change 45 degree of servo motor
						pulse_width=2000;
						pulse_width1=2000;
						pulse_width2=1000;
						pulse_width3=1000;
							
					}
					//rotation
					if(_count>20000){
						Change_flag=1;//while changing
						
						//to rotation
						//write direction
						GPIO_write(Dir_Pin_1,1);
						GPIO_write(Dir_Pin_2,0);
						GPIO_write(Dir_Pin_3,0);
						GPIO_write(Dir_Pin_4,1);

						//stop motor
						motorDIR=GPIO_read(Dir_Pin_1);
						motorPWM=0.95;//rotation
								
						motorDIR_2=GPIO_read(Dir_Pin_2);
						motorPWM_2=0.95;//rotation
						
						motorDIR_3=GPIO_read(Dir_Pin_3);
						motorPWM_3=0.95;//rotation
								
						motorDIR_4=GPIO_read(Dir_Pin_4);
						motorPWM_4=0.95;//rotation

					}
					if(_count>28000){
						//stop RC car
						switch(motorDIR){
							case 0:
								motorPWM=motorDIR;
							break;
							case 1:
								motorPWM=!motorDIR;
						}
						
						switch(motorDIR_2){
							case 0:
								motorPWM_2=motorDIR_2;
							break;
							case 1:
								motorPWM_2=!motorDIR_2;
						}
						switch(motorDIR_3){
							case 0:
								motorPWM_3=motorDIR_3;
							break;
							case 1:
								motorPWM_3=!motorDIR_3;
						}
						switch(motorDIR_4){
							case 0:
								motorPWM_4=motorDIR_4;
							break;
							case 1:
								motorPWM_4=!motorDIR_4;
						}
						//0 degree
						pulse_width=2500;
						pulse_width1=2500;
						pulse_width2=500;
						pulse_width3=600;
					}
					
					//moving back
					if(_count>50000){
						//0 degree
						pulse_width=2500;
						pulse_width1=2500;
						pulse_width2=500;
						pulse_width3=600;
						Back_flag=1;//moving backward
						Change_flag=0;//rotation finish
						
						
						//moving backward
						//write direction
						GPIO_write(Dir_Pin_1,0);
						GPIO_write(Dir_Pin_2,0);
						GPIO_write(Dir_Pin_3,0);
						GPIO_write(Dir_Pin_4,0);
						
						//read direction
						motorDIR=GPIO_read(Dir_Pin_1);
						motorDIR_2=GPIO_read(Dir_Pin_2);
						motorDIR_3=GPIO_read(Dir_Pin_3);
						motorDIR_4=GPIO_read(Dir_Pin_4);
						//_count=0;//initialize count
						
					}
			}
			
			//If the object is detected in the front before the rotation occurs
			if((Ultra_flag==1)&&(Parking_flag==0)&&(fire_flag==0)){
				//read direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);

					//stop RC car
					switch(motorDIR){
						case 0:
							motorPWM=motorDIR;
							motorPWM_4=motorDIR_4;
						break;
						case 1:
							motorPWM=!motorDIR;
							motorPWM_4=!motorDIR_4;
					}
					
					switch(motorDIR_2){
						case 0:
							motorPWM_2=motorDIR_2;
						motorPWM_3=motorDIR_3;
						break;
						case 1:
							motorPWM_2=!motorDIR_2;
						motorPWM_3=!motorDIR_3;
					}
			}
			
			
			//moving backward and detected object(back)
			if((Ultra_flag2==1)&&(Back_flag==1)&&(fire_flag==0)){
				/*
				//change direction, moving forward
						GPIO_write(Dir_Pin_1,1);
						GPIO_write(Dir_Pin_2,1);
						GPIO_write(Dir_Pin_3,1);
						GPIO_write(Dir_Pin_4,1);
						motorDIR=GPIO_read(Dir_Pin_1);
						motorDIR_2=GPIO_read(Dir_Pin_2);
						motorDIR_3=GPIO_read(Dir_Pin_3);
						motorDIR_4=GPIO_read(Dir_Pin_4);
				*/
				//stop RC car
						switch(motorDIR){
							case 0:
								motorPWM=motorDIR;
								motorPWM_4=motorDIR_4;
							break;
							case 1:
								motorPWM=!motorDIR;
								motorPWM_4=!motorDIR_4;
						}
						
						switch(motorDIR_2){
							case 0:
								motorPWM_2=motorDIR_2;
								motorPWM_3=motorDIR_3;
							break;
							case 1:
								motorPWM_2=!motorDIR_2;
								motorPWM_3=!motorDIR_3;
						}
					finish_flag=1;//parking finish
			}

			//If the object is detected from the back before the rotation takes place
			if((Ultra_flag2==1)&&(Parking_flag==0)&&(fire_flag==0)){
				//read direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);
				
				//stop RC car
					switch(motorDIR){
						case 0:
							motorPWM=motorDIR;
							motorPWM_4=motorDIR_4;
						break;
						case 1:
							motorPWM=!motorDIR;
							motorPWM_4=!motorDIR_4;
					}
					
					switch(motorDIR_2){
						case 0:
							motorPWM_2=motorDIR_2;
							motorPWM_3=motorDIR_3;
						break;
						case 1:
							motorPWM_2=!motorDIR_2;
							motorPWM_3=!motorDIR_3;
					}
			}
			
			
			//detect fire after parking 
			if((fire_flag==1)&&(Back_flag==1)&&(Go_fire_flag==0)){
				switch(Ultra_flag2){
					case 1:
						//moving forward when detecting fire
					//write direction
						GPIO_write(Dir_Pin_1,1);
						GPIO_write(Dir_Pin_2,1);
						GPIO_write(Dir_Pin_3,1);
						GPIO_write(Dir_Pin_4,1);
					
					//read direction
						motorDIR=GPIO_read(Dir_Pin_1);
						motorDIR_2=GPIO_read(Dir_Pin_2);
						motorDIR_3=GPIO_read(Dir_Pin_3);
						motorDIR_4=GPIO_read(Dir_Pin_4);
					
					//change duty ratio
						motorPWM=0.8;
						motorPWM_2=0.8;
						motorPWM_3=0.8;
						motorPWM_4=0.8;
						Go_fire_flag=1;//go flag when detecting fire
						break;
					
				}	
			}
			
			if((fire_flag==1)&&(Ultra_flag==1)&&(Turn_flag==0)){
					//detect fire and detect object(front)
					Turn_flag=1;//rotation car
				//read direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);

				//stop RC car
					switch(motorDIR){
					case 0:
						motorPWM=motorDIR;
						motorPWM_4=motorDIR_4;
					break;
					case 1:
						motorPWM=!motorDIR;
						motorPWM_4=!motorDIR_4;
				}
				
				switch(motorDIR_2){
					case 0:
						motorPWM_2=motorDIR_2;
						motorPWM_3=motorDIR_3;
					break;
					case 1:
						motorPWM_2=!motorDIR_2;
						motorPWM_3=!motorDIR_3;
				}
				
		//45 degree of servo motor				
					pulse_width=2000;
					pulse_width1=2000;
					pulse_width2=1000;
					pulse_width3=1000;	
				}
			
			
				//When a fire is detected and rotation is in progress
			if((fire_flag==1)&&(Turn_flag==1)){
				Back_flag=0;//moving forward
				_count1++;
				//before RC car rotation when detecting fire
				if((_count1<10000)&&(Finish_turn_flag==0)){
					//stop RC car
					switch(motorDIR){
					case 0:
						motorPWM=motorDIR;
						motorPWM_4=motorDIR_4;
					break;
					case 1:
						motorPWM=!motorDIR;
						motorPWM_4=!motorDIR_4;
				}
				
				switch(motorDIR_2){
					case 0:
						motorPWM_2=motorDIR_2;
						motorPWM_3=motorDIR_3;
					break;
					case 1:
						motorPWM_2=!motorDIR_2;
						motorPWM_3=!motorDIR_3;
				}
				
				}
				if((_count1>10000)&&(Finish_turn_flag==0)){
					Change_flag=1;//rotation start
					//change direction
					GPIO_write(Dir_Pin_1,1);
					GPIO_write(Dir_Pin_2,0);
					GPIO_write(Dir_Pin_3,0);
					GPIO_write(Dir_Pin_4,1);

					motorDIR=GPIO_read(Dir_Pin_1);
					motorPWM=0.8;//rotation
							
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorPWM_2=0.8;//rotation
					
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorPWM_3=0.8;//rotation
							
					motorDIR_4=GPIO_read(Dir_Pin_4);
					motorPWM_4=0.8;//rotation

					}
					
					if((_count1>24000)&&(Finish_turn_flag==0)){
						
						//stop RC car
						switch(motorDIR){
						case 0:
							motorPWM=motorDIR;
							motorPWM_4=motorDIR_4;
						break;
						case 1:
							motorPWM=!motorDIR;
							motorPWM_4=!motorDIR_4;
					}
					
					switch(motorDIR_2){
						case 0:
							motorPWM_2=motorDIR_2;
							motorPWM_3=motorDIR_3;
						break;
						case 1:
							motorPWM_2=!motorDIR_2;
							motorPWM_3=!motorDIR_3;
					}
					//0 degree of servo motor
						pulse_width=2500;
						pulse_width1=2500;
						pulse_width2=500;
						pulse_width3=600;
				}
					
					
					if((_count1>40000)&&(Finish_turn_flag==0)){
						//0 degree of servo motor
						pulse_width=2500;
						pulse_width1=2500;
						pulse_width2=500;
						pulse_width3=600;
						
						Change_flag=0;//finish rotation
						
						//moving forward
						//write direction
						GPIO_write(Dir_Pin_1,1);
						GPIO_write(Dir_Pin_2,1);
						GPIO_write(Dir_Pin_3,1);
						GPIO_write(Dir_Pin_4,1);
						
						//read direction
						motorDIR=GPIO_read(Dir_Pin_1);
						motorDIR_2=GPIO_read(Dir_Pin_2);
						motorDIR_3=GPIO_read(Dir_Pin_3);
						motorDIR_4=GPIO_read(Dir_Pin_4);
					//	_count1=0;
						Finish_turn_flag=1;//finish rotation when detecting fire
						Ultra_flag=0;//ultra flag is 0
						
					}
				
				if((Ultra_flag==1)&&(Finish_turn_flag==1)){
					//after rotation when detecting fire
					//read direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);

					//stop RC car
						switch(motorDIR){
						case 0:
							motorPWM=motorDIR;
						break;
						case 1:
							motorPWM=!motorDIR;
					}
					
					switch(motorDIR_2){
						case 0:
							motorPWM_2=motorDIR_2;
						break;
						case 1:
							motorPWM_2=!motorDIR_2;
					}
					switch(motorDIR_3){
						case 0:
							motorPWM_3=motorDIR_3;
						break;
						case 1:
							motorPWM_3=!motorDIR_3;
					}
					switch(motorDIR_4){
						case 0:
							motorPWM_4=motorDIR_4;
						break;
						case 1:
							motorPWM_4=!motorDIR_4;
					}
				}
			}
		}
			//right front motor
			duty=fabs(motorDIR-motorPWM);//change speeed
			PWM_duty(Motor_1,duty);//change duty
			
			//left front motor
			duty_2=fabs(motorDIR_2-motorPWM_2);//change speeed
			PWM_duty(Motor_2,duty_2);//change duty
			
			//left back motor
			duty_3=fabs(motorDIR_3-motorPWM_3);//change speeed
			PWM_duty(Motor_3,duty_3);//change duty
			
			//right back motor
			duty_4=fabs(motorDIR_4-motorPWM_4);//change speeed
			PWM_duty(Motor_4,duty_4);//change duty

		//change servo motor pulsewidth
			PWM_pulsewidth_us(Servo_pin,pulse_width);
			PWM_pulsewidth_us(Servo_pin1,pulse_width1);
			PWM_pulsewidth_us(Servo_pin2,pulse_width2);
			PWM_pulsewidth_us(Servo_pin3,pulse_width3);

		// clear by writing 0
		clear_UIF(TIM2); 		// Clear UI flag by writing 0              
	}
}




void TIM5_IRQHandler(void){
	if(is_UIF(TIM5)){ 
		cnt++;//increase cnt for timer interrupt
		cnt1++;//increase cnt1 to read sensor

		//if auto mode
		if((cnt1>100)&&(Mode_flag==1)){
				Ultra_flag=GPIO_read(Ultra_pin1);//read ultra1 flag
				Ultra_flag2=GPIO_read(Ultra_pin2);//read ultra2 flag
				Parking_check_flag=GPIO_read(Parking_pin);//read parking place flag

			switch(Parking_check_flag){
				case 1://if parking check flag is 1->parking flag is 1
					Parking_flag=1;//detect parking place
				break;
			}
			if(value3>2000){
				fire_flag=1;//detect fire
			}
		cnt1=0;//clear cnt1
		
	}
		
		if(((cnt>10)&&(Mode_flag==1)&&(Ultra_flag==0)&&(Ultra_flag2==0)&&(Back_flag==0)&&(Change_flag==0))||((cnt>10)&&(Mode_flag==1)&&(fire_flag==1)&&(Ultra_flag==0)&&(Ultra_flag2==0)&&(Change_flag==0))||((cnt>10)&&(Mode_flag==1)&&(Go_fire_flag==1)&&(Ultra_flag==0)&&(Change_flag==0))){
			
		if((value1<3900)&&(value2<3900)){
			//straight
			motorPWM=1.0;//duty ratio motor1
			motorPWM_2=1.0;//duty ratio motor2
			motorPWM_3=1.0;//duty ratio motor3
			motorPWM_4=1.0;//duty ratio motor4

		}
		else if((value1>3900)&&(value2<3900)){
		//turn right
			motorPWM=0.0;//duty ratio motor1
			motorPWM_2=1.0;//duty ratio motor2
			motorPWM_3=1.0;//duty ratio motor3
			motorPWM_4=0.0;//duty ratio motor4
		}
		else if((value1<3900)&&(value2>3900)){
		//turn left
			motorPWM=1.0;//duty ratio motor1
			motorPWM_2=0.0;//duty ratio motor2
			motorPWM_3=0.0;//duty ratio motor3
			motorPWM_4=1.0;//duty ratio motor4
		}
		else if((value1>3900)&&(value2>3900)){
		//	straight
			motorPWM=1.0;//duty ratio motor1
			motorPWM_2=1.0;//duty ratio motor2
			motorPWM_3=1.0;//duty ratio motor3
			motorPWM_4=1.0;//duty ratio motor4

		}
	cnt=0;//clear interrupt count
	}
		
	//when moving backward
	if((cnt>10)&&(Back_flag==1)){
		if((value1<3900)&&(value2<3900)){
				//straight
					motorPWM=0.7;//\duty ratio motor1
					motorPWM_2=0.7;//0.8 duty ratio motor2
					motorPWM_3=0.7;//0.8 duty ratio motor3
					motorPWM_4=0.7;//0.8 duty ratio motor4
				}
		else if((value1>3900)&&(value2<3900)){
			//turn right
			motorPWM=0.0;//duty ratio motor1
			motorPWM_2=1.0;//duty ratio motor2
			motorPWM_3=1.0;//duty ratio motor3
			motorPWM_4=0.0;//duty ratio motor4
		}
		else if((value1<3900)&&(value2>3900)){
		//turn left
			motorPWM=1.0;//duty ratio motor1
			motorPWM_2=0.0;//duty ratio motor2
			motorPWM_3=0.0;//duty ratio motor3
			motorPWM_4=1.0;//duty ratio motor4
		}
		else if((value1>3900)&&(value2>3900)){
		//straight
			motorPWM=0.7;//duty ratio right front motor
			motorPWM_2=0.7;//duty ratio left front motor
			motorPWM_3=0.7;//duty ratio left back motor
			motorPWM_4=0.7;//duty ratio left bakc motor
		}
	cnt=0;//clear interrupt count
	}

		// clear by writing 0
		clear_UIF(TIM5); 		// Clear UI flag by writing 0              
	}
}



void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
	if(is_ADC_JEOC()){		// after finishing sequence
		value1 = JADC_read(1);//PB0, IR sensor
		value2 = JADC_read(2);//PB1, IR sensor
		value3 = JADC_read(3);//PC0, detect fire(Infrared sensor)
		clear_ADC_JEOC();//clear JEOC
	}
}

