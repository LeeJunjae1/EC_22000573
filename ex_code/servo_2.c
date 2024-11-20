/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-11-15 by JunjaeLee
* @brief   Embedded Controller: Project Line tracing
*
******************************************************************************
*/
#define _CRT_SECURE_NO_WARNINGS  
#include <stdio.h>
#include "stm32f411xe.h"
#include "math.h"
#include "stdlib.h"
//#include "ecSTM32F4v2.h" //if I change my header file, I have to rebuild it. So I don't use it. I check MCU normally running, when I use this header file.

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
//static volatile uint8_t PC_Data = 0;
//static volatile uint8_t BT_Data = 0;
uint8_t PC_string[]="Loop:\r\n";

uint8_t current_mode = 0;//to print current mode
uint8_t current_dir = 0;//tmo print current dir

uint8_t a;
uint8_t b;
uint8_t c;
uint8_t d;

char BT_string[15]={0};//to print sentence
char BT_string1[15]={0};//to print sentence
//Bluetooth PA9, PA10

#define Servo_pin PC_6 //servo   RIGHT
#define Servo_pin1 PC_7    //servo LEFT
#define Servo_pin2 PC_8 //servo   LEFT
#define Servo_pin3 PC_9    //servo RIGHT



#define Motor_1 PA_1 //TIM2 Ch1 Left motor
#define Motor_2 PA_0 //TIM2 Ch2 Right motor
#define Motor_3 PA_5 //TIM2 Ch1 Left motor
#define Motor_4 PB_3 //TIM2 Ch2 Right motor

#define Dir_Pin_1 PC_3 //direction pin
#define Dir_Pin_2 PC_2 //direction pin

#define Mode_pin PA_4 //manual mode->0
#define Change_pin PA_11 //change servo motor->1

#define Go_pin PA_7 //go straight
#define Left_pin PA_12 //turn left
#define Right_pin PA_13 //turn right
#define Stop_pin PA_14 //stop
#define Rotation PA_15 //rotation for parking

#define Ultra_pin1 PB_2 //ultra sensor1
#define Ultra_pin2 PB_4 //ultra sensor2
#define Parking_pin PB_5 //parking flag

unsigned int pulse_width=2500;//calculate pulse width;//calculate pulse width;//pulse width
unsigned int pulse_width1=2500;//calculate pulse width;//calculate pulse width;//pulse width
unsigned int pulse_width2=500;//calculate pulse width;//calculate pulse width;//pulse width
unsigned int pulse_width3=500;//calculate pulse width;//calculate pulse width;//pulse width

//IR parameter//
uint32_t value1, value2;//value1: left ir sensor, value2: right ir sensor
PinName_t seqCHn[2] = {PB_0, PB_1}; //use to pin as jadc

//motor duty
float duty=0.5;
float duty_2=0.5;

volatile uint32_t motorDIR; //motor1 direction
volatile float motorPWM=0.5; //motor1 PWM duty

volatile uint32_t motorDIR_2; //motor2 direction
volatile float motorPWM_2=0.5; //motor2 PWM duty

uint32_t ovf_cnt = 0;//count over count
uint32_t ovf_cnt2 = 0;//count over count
float distance = 0;
float distance2 = 0;

float timeInterval = 0;
float timeInterval2 = 0;
float time1 = 0;//start time
float time2 = 0;//end time

float time1_2 = 0;//start time
float time2_2 = 0;//end time

float time1_3 = 0;//start time
float time2_3 = 0;//end time

float time1_4 = 0;//start time
float time2_4 = 0;//end time

uint32_t ovf_cnt3 = 0;//count over count
uint32_t ovf_cnt4 = 0;//count over count
float distance3 = 0;
float distance4 = 0;

float timeInterval3 = 0;
float timeInterval4 = 0;


volatile uint32_t Ultra_flag=0;// to check object
volatile uint32_t Ultra_flag2=0;// to check object
volatile uint32_t Ultra_flag3=0;// to check object
volatile uint32_t Ultra_flag4=0;// to check object

volatile uint32_t Mode_flag=0;// check mode is auto or manual
volatile uint32_t Change_flag=0;// to check object
volatile uint32_t Parking_flag=0;// to check object

volatile uint32_t Stop_flag=0;// to check stop
volatile uint32_t Go_flag=0;// to check forward
volatile uint32_t Right_flag=0;// to check turn right
volatile uint32_t Left_flag=0;// to check turn left


uint32_t cnt=0;//tim4 interuppt 
uint32_t cnt1=0;//to change led
uint32_t cnt2=0;//to change led


//for change duty
float motorPWMState[4]={0.45f,0.65f,0.85f, 1.0f};// Straight speed
uint8_t PWMstate=0;//motor speed
uint8_t VELstate=0;//VEL state

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
	
	GPIO_init(LED_PIN, OUTPUT);  //led pin
	
	//MODE FLAG
	GPIO_init(Mode_pin, OUTPUT);
	
	GPIO_init(Go_pin, INPUT);  //led pin
	GPIO_init(Right_pin, INPUT);  //led pin
	GPIO_init(Left_pin, INPUT);  //led pin
	GPIO_init(Stop_pin, INPUT);  //led pin
	GPIO_init(Rotation, INPUT);  //led pin
	GPIO_init(Ultra_pin1, INPUT);  //led pin
	GPIO_init(Ultra_pin2, INPUT);  //led pin
	GPIO_init(Parking_pin, INPUT);  //led pin
	
	//motor1
	PWM_init(Motor_1);// pwm settnig
	PWM_period_us(Motor_1,200);//0.2msec, 5kHz
	PWM_duty(Motor_1,duty);//change duty
	
	//motor2
	PWM_init(Motor_2);// pwm settnig
	PWM_period_us(Motor_2,200);//0.2msec, 5kHz
	PWM_duty(Motor_2,duty_2);//change duty
	
	//direction
	GPIO_init(Dir_Pin_1, OUTPUT);  //led pin
	GPIO_init(Dir_Pin_2, OUTPUT);  //led pin
	GPIO_otype(Dir_Pin_1, 0);//push-pull
	GPIO_otype(Dir_Pin_2, 0);//push-pull
	GPIO_write(Dir_Pin_1,1);
	GPIO_write(Dir_Pin_2,0);
	
//read motor dir
	motorDIR=GPIO_read(Dir_Pin_1);
	motorDIR_2=GPIO_read(Dir_Pin_2);
	
	//TIM2
	TIM_UI_enable(TIM2);//TIMER2 enable
	NVIC_EnableIRQ(TIM2_IRQn);	// TIM2's interrupt request enabled	
	NVIC_SetPriority(TIM2_IRQn,3);
	
	
	//100 = 1msec
	PWM_init(Servo_pin);//initialize pwm
	
	// this lab push pull, pull-up, fast
	GPIO_pupd(Servo_pin,1);//pull-up
	GPIO_ospeed(Servo_pin,Fast_speed);//fast
	
	PWM_period_us(Servo_pin,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin,pulse_width);//to change pulse width
	
	
	//100 = 1msec
	PWM_init(Servo_pin1);//initialize pwm
	
		// this lab push pull, pull-up, fast
	GPIO_pupd(Servo_pin1,1);//pull-up
	GPIO_ospeed(Servo_pin1,Fast_speed);//fast
	
	PWM_period_us(Servo_pin1,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin1,pulse_width1);//to change pulse width
	
	//100 = 1msec
	PWM_init(Servo_pin2);//initialize pwm
	
	// this lab push pull, pull-up, fast
	GPIO_pupd(Servo_pin2,1);//pull-up
	GPIO_ospeed(Servo_pin2,Fast_speed);//fast
	
	PWM_period_us(Servo_pin2,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin2,pulse_width2);//to change pulse width
	
	
	//100 = 1msec
	PWM_init(Servo_pin3);//initialize pwm
	
		// this lab push pull, pull-up, fast
	GPIO_pupd(Servo_pin3,1);//pull-up
	GPIO_ospeed(Servo_pin3,Fast_speed);//fast
	
	PWM_period_us(Servo_pin3,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin3,pulse_width3);//to change pulse width
	
	//TIM_UI_init(TIM4, 500);         // TIM3 Update-Event Interrupt every 500 
   //TIM_UI_enable(TIM4); //TIM3 enable
	//NVIC_EnableIRQ(TIM4_IRQn);	// TIM2's interrupt request enabled	
	//NVIC_SetPriority(TIM4_IRQn,4);

}

int main(void){	
	setup();
	printf("Hello Nucleo\r\n");
	USART_write(USART1,(uint8_t*) "Hello bluetooth\r\n",17);
	while(1){
		// USART Receive: Use Interrupt only
		// USART Transmit:  Interrupt or Polling
		//USART2_write(PC_string, 7);
	//	printf("distance1: %f cm\r\n", distance);//display distance
		//printf("distance2: %f cm\r\n", distance2);//display distance
//		printf("time2: \r\n");
	//	printf("ovf1: %d dis1: %f tim1: %f, tim2: %f\r\n",ovf_cnt,distance,time1,time2);
	//	printf("ovf: %d, dis2: %f tim1: %f tim2: %f\r\n\n",ovf_cnt2,distance2,time1_2, time2_2);
		delay_ms(2000);        
	}
}

void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		PC_Data = USART2_read();		// RX from UART2 (PC)
	//	USART_write(USART1,(uint8_t*) "PC sent : ", 10);
		USART_write(USART1,(uint8_t*) "usart2: ",8);
		USART2_write((uint8_t*)&PC_Data,1);		// TX to USART2	 (PC)	 Echo of keyboard typing		
	USART_write(USART1,(uint8_t*) "\r\n",2);
		if(PC_Data=='L'){
			GPIO_write(LED_PIN,0);
			//GPIO_write(PC_0,0);
			//GPIO_write(PC_1,0);
			pulse_width=2000;
			pulse_width1=2000;
			pulse_width2=1000;
			pulse_width3=1000;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			
			Change_flag=1;
			
		}
		else if(PC_Data=='H'){
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			pulse_width=2500;
			pulse_width1=2500;
			pulse_width2=500;
			pulse_width3=500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			

			Change_flag=1;
		}
		
		//for change servo motor
		else if(PC_Data=='K'){
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			pulse_width=1500;
			pulse_width1=1500;
			pulse_width2=1500;
			pulse_width3=1500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2

			Change_flag=1;
		}
	}
}


void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
		BT_Data = USART1_read();		// RX from UART1 (BT)		
		
				//printf("RX: %c \r\n",BT_Data); // TX to USART2(PC)
		USART_write(USART1,(uint8_t*) "RX: ",4);
		USART_write(USART1,(uint8_t*)&BT_Data,1);
		USART_write(USART1,(uint8_t*) "\r\n",2);
		USART_write(USART2,(uint8_t*)&BT_Data,1);
		
		
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
										
										//change speed
										motorPWM=motorPWMState[PWMstate];
										motorPWM_2=motorPWMState[PWMstate];
										
                        break;
										
                    case 'B': // down key
											
										if(PWMstate==0){
											PWMstate=1;
										}
										//speed down
										PWMstate--;
										//change duty
										motorPWM=motorPWMState[PWMstate];
										motorPWM_2=motorPWMState[PWMstate];
                        break;
										
                    case 'C': // right key
											STRval++;
										if(STRval>3){
											//VELstate='3';
											STRval=3; //MAX 3
										}
										STRabs=abs(STRval);
										if(STRval>0){
											//Turn right
										motorPWM=motorPWMState[PWMstate];//left motor speed
										motorPWM_2=motorPWMState[PWMstate]-(0.05f)*(1+STRabs*0.8)*STRabs;//reduce right speed
										}
										else if(STRval==0){
											//Go straight
											motorPWM=motorPWMState[PWMstate];//left motor speed
											motorPWM_2=motorPWMState[PWMstate];

										}
										else if(STRval<0){
											//turn left
											motorPWM=motorPWMState[PWMstate]-(0.05f)*(1+STRabs*0.8)*STRabs;//left motor speed, reduce left speed
											motorPWM_2=motorPWMState[PWMstate];
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
										motorPWM=motorPWMState[PWMstate];//left motor speed
										motorPWM_2=motorPWMState[PWMstate]-(0.05f)*(1+STRabs*0.8)*STRabs;
										}
										else if(STRval==0){
											//Go straight
											motorPWM=motorPWMState[PWMstate];//left motor speed
											motorPWM_2=motorPWMState[PWMstate];
										}
										else if(STRval<0){
											//turn left
											motorPWM=motorPWMState[PWMstate]-(0.05f)*(1+STRabs*0.8)*STRabs;//left motor speed
											motorPWM_2=motorPWMState[PWMstate];

										}
										
                        break;
                    default:
                        break;
									}
								
            }
        } 
				else{
			if((BT_Data=='M')||(BT_Data=='m')){
					current_mode='M';//manual mode
					GPIO_write(LED_PIN,0);//LED off
					GPIO_write(Mode_pin,0);//LED off
					Mode_flag=0;//manual mode
					STRval=0;//str value initialize
					PWMstate=0; //velocity initialize
				}
				else if((BT_Data=='N')||(BT_Data=='n')){
					current_mode='A';//auto mode
					GPIO_write(LED_PIN,1);//LED on, auto mode
					GPIO_write(Mode_pin,1);//LED on
					Mode_flag=1;//auto mode
				}
				
				//moving forward
				else if(((BT_Data=='W')||(BT_Data=='w'))&&(Mode_flag==0)&&(Change_flag==0)){
					current_dir='W';//current dirction is forward
					//Go straight
					GPIO_write(Dir_Pin_1,0);
					GPIO_write(Dir_Pin_2,0);
					
					//read motor dir
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					
					//change motor duty
					motorPWM=motorPWMState[PWMstate];
					motorPWM_2=motorPWMState[PWMstate];

				}
				
				//moving backward
				else if(((BT_Data=='S')||(BT_Data=='s'))&&(Mode_flag==0)&&(Change_flag==0)){
					
					current_dir='S';//current dirction is forward
					//Back
					GPIO_write(Dir_Pin_1,1);//change direction
					GPIO_write(Dir_Pin_2,1);//change direction
					//read motor direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					
					//change motor duty
					motorPWM=motorPWMState[PWMstate];
					motorPWM_2=motorPWMState[PWMstate];
				}
				//stop RC car
				else if(((BT_Data=='F')||(BT_Data=='f'))&&(Mode_flag==0)){
					//read motor direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					if(motorDIR==0){
						//change motor duty
					motorPWM=motorDIR;//stop motor1
					motorPWM_2=motorDIR_2;//stop motor2
					}
					else if(motorDIR==1){
						//change motor duty
						motorPWM=!motorDIR;//stop motor1
						motorPWM_2=!motorDIR_2;//stop motor2
					}
					//STR, VEL initialize
					STRval=0;
					PWMstate=0;
				}
				
				
				//45degree
				else if(BT_Data=='L'){
			GPIO_write(LED_PIN,0);
			//GPIO_write(PC_0,0);
			//GPIO_write(PC_1,0);
			pulse_width=1000;
			pulse_width1=2000;
			pulse_width2=1000;
			pulse_width3=2000;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			
			Change_flag=1;
			
		}
				//parallel
		else if(BT_Data=='H'){
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			pulse_width=500;
			pulse_width1=2500;
			pulse_width2=500;
			pulse_width3=2500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			

			Change_flag=0;// no change servo
		}
		
		//for change servo motor 90 degree
		else if(BT_Data=='J'){
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			pulse_width=1500;
			pulse_width1=1500;
			pulse_width2=1500;
			pulse_width3=1500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2

			Change_flag=1;
		}
		
		//Rotation
		else if(BT_Data=='I'&&(Change_flag==1)){
			GPIO_write(Dir_Pin_1,1);
			GPIO_write(Dir_Pin_2,0);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			//pulse_width=1500;
			//pulse_width1=1500;
			//stop motor
			//motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=0.5;//rotation
					
			//motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=0.5;//rotation

			Change_flag=1;
//			GPIO_write(Change_pin,1);
		}
		else if(BT_Data=='O'&&(Mode_flag==0)){
			GPIO_write(Dir_Pin_1,1);
			GPIO_write(Dir_Pin_2,0);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			//pulse_width=1500;
			//pulse_width1=1500;
			//stop motor
			//motorDIR=GPIO_read(Dir_Pin_1);
			Mode_flag=1;

			Change_flag=1;
//			GPIO_write(Change_pin,1);
		}
		
				
				
		}
		
	}
}

int state=0;//0->up, 1->down
uint32_t _count = 0;//to change 500msec
uint32_t degree = 0;//to change rotation
uint32_t rotation = 1500;//to change rotation degree 45 degree or 90 degree
uint32_t set_degree = 45;//45 degree or 90 degree

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){
		//left motor
		//if (Change_flag==0){
		if(Mode_flag==1){
			Go_flag=GPIO_read(Go_pin);
			Left_flag=GPIO_read(Left_pin);
			Right_flag=GPIO_read(Right_pin);
			Stop_flag=GPIO_read(Stop_pin);
			Ultra_flag=GPIO_read(Ultra_pin1);
			Ultra_flag2=GPIO_read(Ultra_pin2);
			Change_flag=GPIO_read(Parking_pin);
			
			if(Go_flag==1){
				motorPWM=0.8;// left, 0.8 duty ratio motor1
				motorPWM_2=0.8;// left, 0.5 duty ratio motor2
				
			}
			else if(Right_flag==1){
				motorPWM=1.0;// right, 0.5 duty ratio motor1
				motorPWM_2=0.4;// right, 0.8 duty ratio motor2
			}
			else if(Left_flag==1){
				motorPWM=0.4;// left, 0.8 duty ratio motor1
				motorPWM_2=1.0;// left, 0.5 duty ratio motor2
			}
			else if(Stop_flag==1){
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					if(motorDIR==0){
						//change motor duty
					motorPWM=motorDIR;//stop motor1
					motorPWM_2=motorDIR_2;//stop motor2
					}
					else if(motorDIR==1){
						//change motor duty
						motorPWM=!motorDIR;//stop motor1
						motorPWM_2=!motorDIR_2;//stop motor2
					}
			}
			if(Change_flag==1){
				motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					if(motorDIR==0){
						//change motor duty
					motorPWM=motorDIR;//stop motor1
					motorPWM_2=motorDIR_2;//stop motor2
					}
					else if(motorDIR==1){
						//change motor duty
						motorPWM=!motorDIR;//stop motor1
						motorPWM_2=!motorDIR_2;//stop motor2
					}
					pulse_width=1000;
					pulse_width1=2000;
					delay_ms(1000);
					GPIO_write(Dir_Pin_1,1);
					GPIO_write(Dir_Pin_2,0);
					
					motorDIR=GPIO_read(Dir_Pin_1);
					motorPWM=0.5;//rotation
					
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorPWM_2=0.5;//rotation
					
					delay_ms(3000);
					GPIO_write(Dir_Pin_1,1);
					GPIO_write(Dir_Pin_2,1);
					
					motorDIR=GPIO_read(Dir_Pin_1);
					motorPWM=0.5;//rotation
					
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorPWM_2=0.5;//rotation
					
			}
			if(Ultra_flag==1){
				motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					if(motorDIR==0){
						//change motor duty
					motorPWM=motorDIR;//stop motor1
					motorPWM_2=motorDIR_2;//stop motor2
					}
					else if(motorDIR==1){
						//change motor duty
						motorPWM=!motorDIR;//stop motor1
						motorPWM_2=!motorDIR_2;//stop motor2
					}
			}
			if(Ultra_flag2==1){
				motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					if(motorDIR==0){
						//change motor duty
					motorPWM=motorDIR;//stop motor1
					motorPWM_2=motorDIR_2;//stop motor2
					}
					else if(motorDIR==1){
						//change motor duty
						motorPWM=!motorDIR;//stop motor1
						motorPWM_2=!motorDIR_2;//stop motor2
					}
			}
		}
		
		
		duty=fabs(motorDIR-motorPWM);//change speeed
		PWM_duty(Motor_1,duty);//change duty
		
		//right motor
		duty_2=fabs(motorDIR_2-motorPWM_2);//change speeed
		PWM_duty(Motor_2,duty_2);//change duty
		//}
		//else if(Change_flag==1){
			PWM_pulsewidth_us(Servo_pin,pulse_width);//chagne pulse width
			PWM_pulsewidth_us(Servo_pin1,pulse_width1);//chagne pulse width
			PWM_pulsewidth_us(Servo_pin2,pulse_width2);//chagne pulse width
			PWM_pulsewidth_us(Servo_pin3,pulse_width3);//chagne pulse width
				
		//}
		
		// clear by writing 0
		clear_UIF(TIM2); 		// Clear UI flag by writing 0              
	}
	//printf("wefasf\r\n");
}
