/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-11-15 by JunjaeLee
* @brief   Embedded Controller: Final Project
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

#define Servo_pin PC_6 //servo   RIGHT
#define Servo_pin1 PC_7    //servo LEFT
#define Servo_pin2 PC_8 //servo   RIGHT
#define Servo_pin3 PC_9    //servo LEFT

#define Motor_1 PA_1 //TIM2 Ch1 Left motor
#define Motor_2 PA_0 //TIM2 Ch2 Right motor
#define Motor_3 PA_5 //TIM2 Ch1 Left motor
#define Motor_4 PB_3 //TIM2 Ch2 Right motor

#define Dir_Pin_1 PC_3 //direction pin
#define Dir_Pin_2 PC_2 //direction pin

#define TRIG1 PA_8 //pwm
#define ECHO1 PB_6 //echo
#define TRIG2 PA_8 //pwm PB10
#define ECHO2 PB_8 //echo1

#define TRIG3 PA_8 //pwm
#define ECHO3 PA_6 //echo
#define TRIG4 PA_8 //pwm PB10
#define ECHO4 PC_8 //echo1
//bluetooth PA_9, PA_10

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


//#define Ultra_pin3 PB_5 //ultra sensor3
//#define Ultra_pin4 PB_7 //ultra sensor4




//IR parameter//
uint32_t value1, value2;//value1: left ir sensor, value2: right ir sensor
PinName_t seqCHn[2] = {PB_0, PB_1}; //use to pin as jadc
//PC_0 for fire sensor

unsigned int pulse_width=500;//calculate pulse width;//calculate pulse width;//pulse width
unsigned int pulse_width1=2500;//calculate pulse width;//calculate pulse width;//pulse width

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



uint32_t cnt=0;//tim4 interuppt 
uint32_t cnt1=0;//to change led
uint32_t cnt2=0;//to change led


volatile uint32_t Stop_flag=0;// to check stop
volatile uint32_t Go_flag=0;// to check forward
volatile uint32_t Right_flag=0;// to check turn right
volatile uint32_t Left_flag=0;// to check turn left


volatile uint32_t Backward_flag=0;// to check backward
volatile uint32_t auto_flag=0;// to check auto state
volatile uint32_t rotation_45_flag=0;// to check auto state
volatile uint32_t rotation_90_flag=0;// to check auto state
volatile uint32_t rotation_flag=0;// to check auto state
volatile uint32_t Ultra=0;// to check object by ultra1
volatile uint32_t Ultra2=0;// to check object by ultra2
volatile uint32_t Ultra3=0;// to check object by ultra3
volatile uint32_t Ultra4=0;// to check object by ultra4

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
	GPIO_init(Mode_pin, INPUT);
	GPIO_init(Change_pin, INPUT);
	
	GPIO_init(Go_pin, OUTPUT);  //led pin
	GPIO_init(Right_pin, OUTPUT);  //led pin
	GPIO_init(Left_pin, OUTPUT);  //led pin
	GPIO_init(Stop_pin, OUTPUT);  //led pin
	GPIO_init(Rotation, OUTPUT);  //led pin
	GPIO_init(Ultra_pin1, OUTPUT);  //led pin
	GPIO_init(Ultra_pin2, OUTPUT);  //led pin
	GPIO_init(Parking_pin, OUTPUT);  //led pin
	//GPIO_init(Ultra_pin4, OUTPUT);  //led pin
	
	
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
	NVIC_SetPriority(TIM2_IRQn,4);
	
	
	//100 = 1msec
	PWM_init(Servo_pin);//initialize pwm
	
	// this lab push pull, pull-up, fast
	GPIO_pupd(Servo_pin,1);//pull-up
	GPIO_ospeed(Servo_pin,Fast_speed);//fast
	
	PWM_period_us(Servo_pin,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin,pulse_width);//to change pulse width
	
	/*
	//100 = 1msec
	PWM_init(Servo_pin1);//initialize pwm
	
		// this lab push pull, pull-up, fast
	GPIO_pupd(Servo_pin1,1);//pull-up
	GPIO_ospeed(Servo_pin1,Fast_speed);//fast
	
	PWM_period_us(Servo_pin1,20000);//period 20msec
	PWM_pulsewidth_us(Servo_pin1,pulse_width1);//to change pulse width
	
	
	NVIC_EnableIRQ(TIM4_IRQn);	// TIM2's interrupt request enabled	
	NVIC_SetPriority(TIM4_IRQn,4);
	*/
	
	
	//INIT ultra sonic
	GPIO_otype(TRIG1, 0);//push pull
	GPIO_pupd(TRIG1,0);//NO pull-up pull-down
	GPIO_ospeed(TRIG1,Fast_speed);//FAST SPEED
	
	
	// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG1);			// PA_6: Ultrasonic trig pulse
	//PWM_period_ms(TRIG1, 1);    // PWM of 50ms period. Use period_us()
	PWM_period_us(TRIG1, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG1, 10);   // PWM pulse width of 10us

	// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO1);    	// PB_6 as input caputre
	GPIO_pupd(ECHO1,0);//NO pull-up pull-down
 	ICAP_counter_us(ECHO1, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO1, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO1, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect


/*
	//INIT ultra sonic
	GPIO_otype(TRIG2, 0);//push pull
	GPIO_pupd(TRIG2,0);//NO pull-up pull-down
	GPIO_ospeed(TRIG2,Fast_speed);//FAST SPEED

	PWM_init(TRIG2);			// PA_6: Ultrasonic trig pulse
//	PWM_period_ms(TRIG2, 1);    // PWM of 50ms period. Use period_us()
	PWM_period_us(TRIG2, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG2, 10);   // PWM pulse width of 10us
*/	
	//Input Capture 2 
	ICAP_init(ECHO2);    	// PB_6 as input caputre
	GPIO_pupd(ECHO2,0);//NO pull-up pull-down
 	ICAP_counter_us(ECHO2, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO2, 3, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO2, 4, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
	NVIC_SetPriority(TIM4_IRQn,3);
/*
	//Input Capture 3
// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG3);			// PA_6: Ultrasonic trig pulse
	//PWM_period_ms(TRIG1, 1);    // PWM of 50ms period. Use period_us()
	PWM_period_us(TRIG3, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG3, 10);   // PWM pulse width of 10us
*/
	// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO3);    	// PB_6 as input caputre
	GPIO_pupd(ECHO3,0);//NO pull-up pull-down
 	ICAP_counter_us(ECHO3, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO3, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO3, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
	
/*

	//INIT ultra sonic
	GPIO_otype(TRIG4, 0);//push pull
	GPIO_pupd(TRIG4,0);//NO pull-up pull-down
	GPIO_ospeed(TRIG4,Fast_speed);//FAST SPEED

	PWM_init(TRIG4);			// PA_6: Ultrasonic trig pulse
//	PWM_period_ms(TRIG2, 1);    // PWM of 50ms period. Use period_us()
	PWM_period_us(TRIG4, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG4, 10);   // PWM pulse width of 10us
	*/
	//Input Capture 4 
	ICAP_init(ECHO4);    	// PB_6 as input caputre
	GPIO_pupd(ECHO4,0);//NO pull-up pull-down
 	ICAP_counter_us(ECHO4, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO4, 3, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO4, 4, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
//	NVIC_SetPriority(TIM3_IRQn,3);


// ADC Init  Default: HW triggered by TIM3 counter @ 1msec
	//change tim3->tim5
	JADC_init(PB_0);
	JADC_init(PB_1);

	// ADC channel sequence setting
	JADC_sequence(seqCHn, 2);
	
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
		// USART Receive: Use Interrupt only
		// USART Transmit:  Interrupt or Polling
		//USART2_write(PC_string, 7);
	//	printf("distance1: %f cm\r\n", distance);//display distance
	//	printf("distance2: %f cm\r\n\n", distance2);//display distance
		printf("distance3: %f cm\r\n", distance3);//display distance
		printf("distance4: %f cm\r\n\n", distance4);//display distance
		printf("value1 = %d \r\n",value1);
		printf("value2 = %d \r\n",value2);
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
			//degree 45
			pulse_width=1000;
			pulse_width1=2000;
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
			//degree 0
			pulse_width=500;
			pulse_width1=2500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			

			Change_flag=1;
		}
		
		//for change servo motor
		else if(PC_Data=='J'){
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			//degree 90
			pulse_width=1500;
			pulse_width1=1500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2

			Change_flag=1;
		}
		else if(PC_Data=='N'||PC_Data=='n'){
			//auto mode
			Mode_flag=1;
			pulse_width=500;
			pulse_width1=2500;
			Change_flag=0;
		}
		else if(PC_Data=='M'||PC_Data=='m'){
			//manual mode
			Mode_flag=0;
			pulse_width=500;
			pulse_width1=2500;
			Change_flag=0;
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
					GPIO_write(LED_PIN,1);//LED ON
					Mode_flag=0;//manual mode
					STRval=0;//str value initialize
					PWMstate=0; //velocity initialize
				}
				else if((BT_Data=='N')||(BT_Data=='n')){
					current_mode='A';//auto mode
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
				else if(((BT_Data=='F')||(BT_Data=='f'))&&(Mode_flag==0)&&(Change_flag==0)){
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
				
				
				else if(BT_Data=='L'){
		//	GPIO_write(LED_PIN,0);
			//GPIO_write(PC_0,0);
			//GPIO_write(PC_1,0);
			pulse_width=1000;
			pulse_width1=2000;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			
			Change_flag=1;
			
		}
		else if(BT_Data=='H'){
			//GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			pulse_width=500;
			pulse_width1=2500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			

			Change_flag=0;// no change servo
		}
		
		//for change servo motor
		else if(BT_Data=='J'){
		//	GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			pulse_width=1500;
			pulse_width1=1500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2

			Change_flag=1;
		}
		else if(BT_Data=='I'&&(Change_flag==1)){
			GPIO_write(Dir_Pin_1,1);
			GPIO_write(Dir_Pin_2,0);

			motorPWM=0.5;//rotation
			motorPWM_2=0.5;//rotation

			Change_flag=1;
		}
		
				
				
		}
		
	}
}

/*
int state=0;//0->up, 1->down
uint32_t _count = 0;//to change 500msec
uint32_t degree = 0;//to change rotation
uint32_t rotation = 1500;//to change rotation degree 45 degree or 90 degree
uint32_t set_degree = 45;//45 degree or 90 degree*/




void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){                     // Update interrupt
		ovf_cnt3++;													// overflow count
		ovf_cnt4++;													// overflow count
		clear_UIF(TIM3);  							    // clear update interrupt flag
	}
	
	if(is_CCIF(TIM3, 1)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1_3 = ICAP_capture(TIM3,1);									// Capture TimeStart
		clear_CCIF(TIM3, 1);                // clear capture/compare interrupt flag 
	}								                      
	else if(TIM3,2){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2_3 = ICAP_capture(TIM3,2);									// Capture TimeEnd
		timeInterval3 = ((time2_3-time1_3)+ovf_cnt3*((TIM3->ARR)+1))/100.0; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		distance3 = (float) timeInterval3 * 340.0 / 2.0 /10.0; 	// [mm] -> [cm]
		ovf_cnt3 = 0;                        // overflow reset
		
		if(((distance3>50)&&(distance3<0))&&(Mode_flag==1)){
			Ultra_flag3=1;// detect object
			//GPIO_write(Ultra_pin3, 1);//ultra flag
		}
		else{
			Ultra_flag3=0;//no object
			//GPIO_write(Ultra_pin3, 0);//ultra flag no

		}
		clear_CCIF(TIM3,2);								  // clear capture/compare interrupt flag 
	}
	
	if(is_CCIF(TIM3, 3)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1_4 = ICAP_capture(TIM3,3);									// Capture TimeStart
		clear_CCIF(TIM3, 3);                // clear capture/compare interrupt flag 
	}								                      
	else if(TIM3,4){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2_4 = ICAP_capture(TIM3,4);									// Capture TimeEnd
		timeInterval4 = ((time2_4-time1_4)+ovf_cnt4*((TIM3->ARR)+1))/100.0; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		distance4 = (float) timeInterval4 * 340.0 / 2.0 /10.0; 	// [mm] -> [cm]
		ovf_cnt4 = 0;                        // overflow reset
		
		if(((distance4>50)&&(distance4<0))&&(Mode_flag==1)){
			Ultra_flag4=1;// detect object
			//GPIO_write(Ultra_pin4, 1);//ultra flag4 high
			
			if((Ultra_flag3==1)&&(Ultra_flag4==1)){
			//detect parking space
				Change_flag=1;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
				GPIO_write(Stop_pin,1);
				GPIO_write(Change_pin,1);
				/*
				pulse_width=1000;
			pulse_width1=2000;
				delay_ms(1000);
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
			delay_ms(3000);
			GPIO_write(Dir_Pin_1,1);
			GPIO_write(Dir_Pin_2,1);*/
			}
		}
		else{
			Ultra_flag4=0;//no object
			//GPIO_write(Ultra_pin4,0);//no ultra flag4
			//GPIO_write(Change_pin,0);
		}
		clear_CCIF(TIM3,4);								  // clear capture/compare interrupt flag 
	}
}



void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                     // Update interrupt
		ovf_cnt++;													// overflow count
		ovf_cnt2++;													// overflow count
		clear_UIF(TIM4);  							    // clear update interrupt flag
	}
	
	if(is_CCIF(TIM4, 1)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4,1);									// Capture TimeStart
		clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag 
	}								                      
	else if(TIM4,2){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2 = ICAP_capture(TIM4,2);									// Capture TimeEnd
		timeInterval = ((time2-time1)+ovf_cnt*((TIM4->ARR)+1))/100.0; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		distance = (float) timeInterval * 340.0 / 2.0 /10.0; 	// [mm] -> [cm]
		ovf_cnt = 0;                        // overflow reset
		
		if(((distance<10)&&(distance>0))&&(Mode_flag==1)){
			//forward
			Ultra_flag=1;// detect object
			GPIO_write(Ultra_pin1, 1);//ultra 1 high
			GPIO_write(Stop_pin,1);//stop rc car
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			
		}
		else{
			Ultra_flag=0;//no object
			GPIO_write(Ultra_pin1,0);
		}
		clear_CCIF(TIM4,2);								  // clear capture/compare interrupt flag 
	}
	
	if(is_CCIF(TIM4, 3)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1_2 = ICAP_capture(TIM4,3);									// Capture TimeStart
		clear_CCIF(TIM4, 3);                // clear capture/compare interrupt flag 
	}								                      
	else if(TIM4,4){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2_2 = ICAP_capture(TIM4,4);									// Capture TimeEnd
		timeInterval2 = ((time2_2-time1_2)+ovf_cnt2*((TIM4->ARR)+1))/100.0; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		distance2 = (float) timeInterval2 * 340.0 / 2.0 /10.0; 	// [mm] -> [cm]
		ovf_cnt2 = 0;                        // overflow reset
		
		if(((distance2<10)&&(distance2>0))&&(Mode_flag==1)){
			//detect backward
			Ultra_flag2=1;// detect object
			
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=motorDIR_2;//stop motor2
			GPIO_write(Ultra_pin2,1);
			GPIO_write(Stop_pin,1);
		}
		else{
			Ultra_flag2=0;//no object
			GPIO_write(Ultra_pin2,0);
		}
		clear_CCIF(TIM4,4);								  // clear capture/compare interrupt flag 
	}
}

void TIM5_IRQHandler(void){
	if(is_UIF(TIM5)){ 
		cnt++;//timer interrupt
		cnt1++;//for led count
		Mode_flag=GPIO_read(Mode_pin);
		//Change_flag=GPIO_read(Change_pin);
		if((cnt>10)&&(Mode_flag==1)&&(Ultra_flag==0)&&(Change_flag==0)){
			
		if((value1<1700)&&(value2<1700)){
			motorPWM=1.0;//0.8 duty ratio motor1
			motorPWM_2=1.0;//0.8 duty ratio motor2
			GPIO_write(Go_pin,1);
			GPIO_write(Left_pin,0);
			GPIO_write(Right_pin,0);
			GPIO_write(Stop_pin,0);
		//	STRval=0;
			//STRstate='0';//go straight
		}
		else if((value1>1700)&&(value2<1700)){
			motorPWM=0.4;// left, 0.8 duty ratio motor1
			motorPWM_2=1.0;// left, 0.5 duty ratio motor2
			GPIO_write(Go_pin,0);
			GPIO_write(Left_pin,1);
			GPIO_write(Right_pin,0);
			GPIO_write(Stop_pin,0);
	//		STRval=-1;//left
		//	STRstate='1';//turn left
		}
		else if((value1<1700)&&(value2>1700)){
			motorPWM=1.0;// right, 0.5 duty ratio motor1
			motorPWM_2=0.4;// right, 0.8 duty ratio motor2
			GPIO_write(Go_pin,0);
			GPIO_write(Left_pin,0);
			GPIO_write(Right_pin,1);
			GPIO_write(Stop_pin,0);
	//		STRval=1;//right
		//	STRstate='1';//turn right
		}
		else if((value1>1700)&&(value2>1700)){
			motorPWM=1.0;//0.8 duty ratio motor1
			motorPWM_2=1.0;//0.8 duty ratio motor2
			GPIO_write(Go_pin,1);
			GPIO_write(Left_pin,0);
			GPIO_write(Right_pin,0);
			GPIO_write(Stop_pin,0);
	//		STRval=0;
		//	STRstate='0';//go straight
		}
		
	//	VELstate='1';//print vel value is 1
	cnt=0;//clear interrupt count
	}
		if(cnt1>2000&&(Mode_flag==1)&&((Ultra_flag==0)||(Ultra_flag2==0))&&(Change_flag==0)){
			//auto mode 2sec blink
			LED_toggle();
			cnt1=0;//clear led count
		}
		else if(cnt1>500&&(Mode_flag==1)&&((Ultra_flag==1)||(Ultra_flag2==1))&&(Change_flag==0)){
			//auto mode 2sec blink
			LED_toggle();
			cnt1=0;//clear led count
		}
		// clear by writing 0
		clear_UIF(TIM5); 		// Clear UI flag by writing 0              
	}
}



void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
	if(is_ADC_JEOC()){		// after finishing sequence
		value1 = JADC_read(1);//PB0
		value2 = JADC_read(2);//PB1
		clear_ADC_JEOC();//clear JEOC
	}
}