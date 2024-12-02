/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-11-15 by JunjaeLee
* @brief   Embedded Controller: Final Project RC car
*
******************************************************************************
*/
//before fire flag working

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
//uint8_t PC_string[]="Loop:\r\n";

//uint8_t current_mode = 0;//to print current mode
//uint8_t current_dir = 0;//tmo print current dir


//char BT_string[15]={0};//to print sentence
//char BT_string1[15]={0};//to print sentence
//Bluetooth PA9, PA10

#define Servo_pin PC_6 //servo   Left backward
#define Servo_pin1 PC_7    //servo Right Forward
#define Servo_pin2 PC_8 //servo   RighT backward
#define Servo_pin3 PC_9    //servo left forward



#define Motor_1 PA_1 //TIM2 Ch1 Forward RIGHT motor
#define Motor_2 PA_0 //TIM2 Ch2 Forward LEFT motor
#define Motor_3 PA_5 //TIM2 Ch1 Back Left motor
#define Motor_4 PB_3 //TIM2 Ch2 Back Right motor

#define Dir_Pin_1 PC_3 //direction pin forwrd RIGHT
#define Dir_Pin_2 PC_2 //direction pin forward LEFT
#define Dir_Pin_3 PC_13 //direction pin back left
#define Dir_Pin_4 PB_7 //direction pin back right

#define Mode_pin PA_4 //manual mode->0
#define Change_pin PA_11 //change servo motor->1

//#define Go_pin PA_7 //go straight
//#define Left_pin PA_12 //turn left
//#define Right_pin PA_13 //turn right
//#define Stop_pin PA_14 //stop
//#define Rotation PB_15 //rotation for parking

#define Ultra_pin1 PB_2 //ultra sensor1
#define Ultra_pin2 PB_4 //ultra sensor2
#define Parking_pin PB_5 //parking flag
//#define Parking_finish_pin PB_14 //manual mode->0

//#define Fire_pin PB_10 //fire flag 

unsigned int pulse_width=2500;//calculate pulse width;//calculate pulse width;//pulse width
unsigned int pulse_width1=2500;//calculate pulse width;//calculate pulse width;//pulse width
unsigned int pulse_width2=500;//calculate pulse width;//calculate pulse width;//pulse width
unsigned int pulse_width3=500;//calculate pulse width;//calculate pulse width;//pulse width

//IR parameter//
uint32_t value1, value2, value3;//value1: left ir sensor, value2: right ir sensor
PinName_t seqCHn[3] = {PB_0, PB_1, PC_4}; //use to pin as jadc
//PC_0 for sensor


volatile uint32_t motorDIR; //motor1 direction
volatile float motorPWM=0.6; //motor1 PWM duty

volatile uint32_t motorDIR_2; //motor2 direction
volatile float motorPWM_2=0.6; //motor2 PWM duty

volatile uint32_t motorDIR_3; //motor3 direction
volatile float motorPWM_3=0.6; //motor3 PWM duty

volatile uint32_t motorDIR_4; //motor4 direction
volatile float motorPWM_4=0.6; //motor4 PWM duty

//motor duty
float duty=1.0;
float duty_2=1.0;
float duty_3=1.0;
float duty_4=1.0;



volatile uint32_t Ultra_flag=0;// to check object
volatile uint32_t Ultra_flag2=0;// to check object
//volatile uint32_t Ultra_flag3=0;// to check object
//volatile uint32_t Ultra_flag4=0;// to check object

volatile uint32_t Mode_flag=0;// check mode is auto or manual
volatile uint32_t Change_flag=0;// to check object
volatile uint32_t Parking_flag=0;// to check object
volatile uint32_t Parking_check_flag=0;// to check object
//volatile uint32_t Ultra_check_flag=0;// to check object
//volatile uint32_t Ultra_check_flag2=0;// to check object


volatile uint32_t Back_flag=0;// to check object
volatile uint32_t finish_flag=0;// to check object
volatile uint32_t fire_flag=0;// to check object
volatile uint32_t Go_fire_flag=0;// to moving fire
volatile uint32_t Turn_flag=0;// to turn rc car
volatile uint32_t Finish_turn_flag=0;// to turn rc car



//volatile uint32_t Stop_flag=0;// to check stop
//volatile uint32_t Go_flag=0;// to check forward
//volatile uint32_t Right_flag=0;// to check turn right
//volatile uint32_t Left_flag=0;// to check turn left

//volatile uint32_t Servo_flag=0;// to change servo duty

uint32_t cnt=0;//tim4 interuppt 
uint32_t cnt1=0;//to change led
//uint32_t cnt2=0;//to change led

//uint32_t cnt_ultra=0;//to change led
//uint32_t cnt_ultra2=0;//to change led

//for change duty
float motorPWMState[4]={0.5f,0.65f,0.85f, 1.0f};// Straight speed
uint8_t PWMstate=0;//motor speed
//uint8_t VELstate=0;//VEL state

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
	
//	GPIO_init(LED_PIN, OUTPUT);  //led pin
	

	
	//TIM_UI_init(TIM4, 500);         // TIM3 Update-Event Interrupt every 500 
//   TIM_UI_enable(TIM4); //TIM3 enable
//	NVIC_EnableIRQ(TIM4_IRQn);	// TIM2's interrupt request enabled	
//	NVIC_SetPriority(TIM4_IRQn,3);

// ADC Init  Default: HW triggered by TIM3 counter @ 1msec
	//change tim3->tim5
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
		// USART Receive: Use Interrupt only
		// USART Transmit:  Interrupt or Polling
		//USART2_write(PC_string, 7);
	//	printf("distance1: %f cm\r\n", distance);//display distance
		//printf("distance2: %f cm\r\n", distance2);//display distance
//		printf("time2: \r\n");
	//	printf("ovf1: %d dis1: %f tim1: %f, tim2: %f\r\n",ovf_cnt,distance,time1,time2);
	//	printf("ovf: %d, dis2: %f tim1: %f tim2: %f\r\n\n",ovf_cnt2,distance2,time1_2, time2_2);
		
		
		printf("val1: %d val2: %d fire: %d\r\n",value1, value2, value3);
		delay_ms(1000);        
	}
}

void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		PC_Data = USART2_read();		// RX from UART2 (PC)
	//	USART_write(USART1,(uint8_t*) "PC sent : ", 10);
		USART_write(USART1,(uint8_t*) "usart2: ",8);
		USART2_write((uint8_t*)&PC_Data,1);		// TX to USART2	 (PC)	 Echo of keyboard typing		
	USART_write(USART1,(uint8_t*) "\r\n",2);
		
		//45degree
		if(PC_Data=='L'){
	//		Servo_flag=1;
		//	GPIO_write(LED_PIN,0);
			//GPIO_write(PC_0,0);
			//GPIO_write(PC_1,0);
			pulse_width=2000;
			pulse_width1=2000;
			pulse_width2=1000;
			pulse_width3=1000;
			
//			PWM_pulsewidth_us(Servo_pin,pulse_width);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin1,pulse_width1);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin2,pulse_width2);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin3,pulse_width3);//chagne pulse width
			
			
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			//motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			//motorPWM_2=motorDIR_2;//stop motor2
			
			//stop motor
			motorDIR_3=GPIO_read(Dir_Pin_3);
			//motorPWM_3=motorDIR_3;//stop motor1
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
			//motorPWM_4=motorDIR_4;//stop motor2
			
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
			Change_flag=1;
			
		}
		//90 degree
		else if(PC_Data=='H'){
	//		Servo_flag=1;
			//GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			pulse_width=1500;
			pulse_width1=1500;
			pulse_width2=1500;
			pulse_width3=1500;
			
//			PWM_pulsewidth_us(Servo_pin,pulse_width);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin1,pulse_width1);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin2,pulse_width2);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin3,pulse_width3);//chagne pulse width
			
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			//motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			//motorPWM_2=motorDIR_2;//stop motor2
			
			//stop motor
			motorDIR_3=GPIO_read(Dir_Pin_3);
			//motorPWM_3=motorDIR_3;//stop motor1
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
			//motorPWM_4=motorDIR_4;//stop motor2
			
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
			Change_flag=1;
		}
		
		//for change servo motor PALLEL degree
		else if(PC_Data=='K'){
		//	GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
	//		Servo_flag=1;
			pulse_width=2500;
			pulse_width1=2500;
			pulse_width2=500;
			pulse_width3=500;
//			PWM_pulsewidth_us(Servo_pin,pulse_width);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin1,pulse_width1);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin2,pulse_width2);//chagne pulse width
//			PWM_pulsewidth_us(Servo_pin3,pulse_width3);//chagne pulse width
			
			
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			//motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			//motorPWM_2=motorDIR_2;//stop motor2
			
			//stop motor
			motorDIR_3=GPIO_read(Dir_Pin_3);
			//motorPWM_3=motorDIR_3;//stop motor1
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
			//motorPWM_4=motorDIR_4;//stop motor2
			
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

			Change_flag=0;
		}
		else if(PC_Data=='I'&&(Change_flag==1)){
			GPIO_write(Dir_Pin_1,1);
			GPIO_write(Dir_Pin_2,0);
			GPIO_write(Dir_Pin_3,0);
			GPIO_write(Dir_Pin_4,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			//pulse_width=1500;
			//pulse_width1=1500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=0.5;//rotation
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=0.5;//rotation
			
			motorDIR_3=GPIO_read(Dir_Pin_3);
			motorPWM_3=0.5;//rotation
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
			motorPWM_4=0.5;//rotation

			Change_flag=1;
//			GPIO_write(Change_pin,1);
		}
		else if(PC_Data=='A'){
			Mode_flag=1;
		}
		
		else{
		//	Servo_flag=0;
			Mode_flag=0;
			motorPWM=0.8;//rotation
					
			//motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=0.8;//rotation
			
			motorPWM_3=0.8;//rotation
					
			//motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_4=0.8;//rotation
		}
	}
}


void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART_RXNE(USART1)){
		BT_Data = USART_read(USART1);		// RX from UART1 (BT)		
		
				//printf("RX: %c \r\n",BT_Data); // TX to USART2(PC)
	//	USART_write(USART1,(uint8_t*) "RX: ",4);
	//	USART_write(USART1,(uint8_t*)&BT_Data,1);
		//USART_write(USART1,(uint8_t*) "\r\n",2);
		//USART_write(USART2,(uint8_t*)&BT_Data,1);
		
		
		//check direction key
		if (BT_Data == 0x1B) { // ESC key
		//	printf("bt 1: %d\r\n",BT_Data);
            uint8_t nextChar1 = USART_read(USART1);
		//	printf("next 1: %d\r\n",nextChar1);
            if (nextChar1 == '[') {
                uint8_t nextChar2 = USART_read(USART1);
                switch (nextChar2) {
                    case 'A': // up key
											PWMstate++;//Speed up
										if(PWMstate>3){
											PWMstate=3;
										}
										//printf("PWM %d\r\n",PWMstate);
										//change speed
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
										//change duty
										motorPWM=motorPWMState[PWMstate];
										motorPWM_2=motorPWMState[PWMstate];
										motorPWM_3=motorPWMState[PWMstate];
										motorPWM_4=motorPWMState[PWMstate];
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
										motorPWM=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left motor speed
										motorPWM_2=motorPWMState[PWMstate];//reduce right speed
										motorPWM_3=motorPWMState[PWMstate];//left motor speed
										motorPWM_4=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//reduce right speed
										
										}
										else if(STRval==0){
											//Go straight
											motorPWM=motorPWMState[PWMstate];//left motor speed
											motorPWM_2=motorPWMState[PWMstate];
											motorPWM_3=motorPWMState[PWMstate];//left motor speed
											motorPWM_4=motorPWMState[PWMstate];

										}
										else if(STRval<0){
											//turn left
											motorPWM=motorPWMState[PWMstate];//left motor speed, reduce left speed
											motorPWM_2=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;
											motorPWM_3=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left motor speed, reduce left speed
											motorPWM_4=motorPWMState[PWMstate];
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
										motorPWM=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left motor speed
										motorPWM_2=motorPWMState[PWMstate];
										motorPWM_3=motorPWMState[PWMstate];//left motor speed
										motorPWM_4=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;
										
										}
										else if(STRval==0){
											//Go straight
											motorPWM=motorPWMState[PWMstate];//left motor speed
											motorPWM_2=motorPWMState[PWMstate];
											motorPWM_3=motorPWMState[PWMstate];//left motor speed
											motorPWM_4=motorPWMState[PWMstate];
										
										}
										else if(STRval<0){
											//turn left
											motorPWM=motorPWMState[PWMstate];
											motorPWM_2=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left motor speed
											motorPWM_3=motorPWMState[PWMstate]-(0.11f)*(1+STRabs*0.8)*STRabs;//left motor speed
											motorPWM_4=motorPWMState[PWMstate];

										}
										
                        break;
                    default:
                        break;
									}
								
            }
        } 
				else{
			if((BT_Data=='M')||(BT_Data=='m')){
					Change_flag=0;
	//				current_mode='M';//manual mode
					//GPIO_write(LED_PIN,0);//LED off
					GPIO_write(Mode_pin,0);//LED off
					Mode_flag=0;//manual mode
					STRval=0;//str value initialize
					PWMstate=0; //velocity initialize
				fire_flag=0;
				Change_flag=0;
				Ultra_flag=0;
				Ultra_flag2=0;
				Parking_flag=0;
				Back_flag=0;
				}
				else if((BT_Data=='N')||(BT_Data=='n')){
	//				current_mode='A';//auto mode
					//GPIO_write(LED_PIN,1);//LED on, auto mode
					GPIO_write(Mode_pin,1);//LED on
					Mode_flag=1;//auto mode
					Change_flag=0;
					Ultra_flag=0;
					Ultra_flag2=0;
					Parking_flag=0;
					finish_flag=0;
				}
				
				//moving forward
				else if(((BT_Data=='W')||(BT_Data=='w'))&&(Mode_flag==0)&&(Change_flag==0)){
				//	Change_flag=0;
		//			current_dir='W';//current dirction is forward
					
		//			pulse_width=2500;
			pulse_width1=2500;
	//		pulse_width2=500;
			pulse_width3=500;
					
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
					/*
					PWMstate++;//Speed up
					if(PWMstate>3){
											PWMstate=3;
										}
										printf("up\r\n");
										//printf("PWM %d\r\n",PWMstate);
										//change speed
										motorPWM=motorPWMState[PWMstate];
										motorPWM_2=motorPWMState[PWMstate];
										motorPWM_3=motorPWMState[PWMstate];
										motorPWM_4=motorPWMState[PWMstate];*/
					

				}
				
				//moving backward
				else if(((BT_Data=='S')||(BT_Data=='s'))&&(Mode_flag==0)&&(Change_flag==0)){
			//		Change_flag=0;
	//				current_dir='S';//current dirction is forward
		//			pulse_width=2500;
			pulse_width1=2500;
	//		pulse_width2=500;
			pulse_width3=500;
					
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
			//		Change_flag=0;
		//			current_dir='S';//current dirction is forward
					
				//	pulse_width=2000;
//			pulse_width1=2000; //USE
		//	pulse_width2=1000; 
//			pulse_width3=1000; //USE
					
					//Back
			//		GPIO_write(Dir_Pin_1,0);//change direction
		//			GPIO_write(Dir_Pin_2,0);//change direction
					//read motor direction
			//		motorDIR=GPIO_read(Dir_Pin_1);
		//			motorDIR_2=GPIO_read(Dir_Pin_2);
					
		//			GPIO_write(Dir_Pin_3,0);//change direction
			//		GPIO_write(Dir_Pin_4,0);//change direction
					//read motor direction
			//		motorDIR_3=GPIO_read(Dir_Pin_3);
			//		motorDIR_4=GPIO_read(Dir_Pin_4);
					
					//change motor duty
					motorPWM=1.0;
					motorPWM_2=0.3;
					motorPWM_3=0.3;
					motorPWM_4=1.0;
				}
				else if(((BT_Data=='D')||(BT_Data=='d'))&&(Mode_flag==0)&&(Change_flag==0)){
			//		Change_flag=0;
		//			current_dir='D';//current dirction is forward
				//	pulse_width=2000;
//			pulse_width1=2000; //USE
		//	pulse_width2=1000;
//			pulse_width3=1000; //USE
					
					//Back
				//	GPIO_write(Dir_Pin_1,0);//change direction
				//	GPIO_write(Dir_Pin_2,0);//change direction
					//read motor direction
				//	motorDIR=GPIO_read(Dir_Pin_1);
			//		motorDIR_2=GPIO_read(Dir_Pin_2);
					
			//		GPIO_write(Dir_Pin_3,0);//change direction
			//		GPIO_write(Dir_Pin_4,0);//change direction
					//read motor direction
			//		motorDIR_3=GPIO_read(Dir_Pin_3);
			//		motorDIR_4=GPIO_read(Dir_Pin_4);
					
					//change motor duty
					motorPWM=0.3;
					motorPWM_2=1.0;
					motorPWM_3=1.0;
					motorPWM_4=0.3;
				}
				//stop RC car
				else if(((BT_Data=='F')||(BT_Data=='f'))&&(Mode_flag==0)){
					//read motor direction
					motorDIR=GPIO_read(Dir_Pin_1);
					motorDIR_2=GPIO_read(Dir_Pin_2);
					motorDIR_3=GPIO_read(Dir_Pin_3);
					motorDIR_4=GPIO_read(Dir_Pin_4);
					
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
		//	GPIO_write(LED_PIN,0);
			//GPIO_write(PC_0,0);
			//GPIO_write(PC_1,0);
			pulse_width=2000;
			pulse_width1=2000;
			pulse_width2=1000;
			pulse_width3=1000;
					
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
	//		motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
		///	motorPWM_2=motorDIR_2;//stop motor2
			
			//stop motor
			motorDIR_3=GPIO_read(Dir_Pin_3);
	//		motorPWM_3=motorDIR_3;//stop motor1
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
	//		motorPWM_4=motorDIR_4;//stop motor2
					
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
			Change_flag=1;
			
		}
				//90 degree
		else if(BT_Data=='H'){
		//	GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			
			pulse_width=1500;
			pulse_width1=1500;
			pulse_width2=1500;
			pulse_width3=1500;
			
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
		//	motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
		//	motorPWM_2=motorDIR_2;//stop motor2
			
			//stop motor
			motorDIR_3=GPIO_read(Dir_Pin_3);
		//	motorPWM_3=motorDIR_3;//stop motor1
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
		//	motorPWM_4=motorDIR_4;//stop motor2
			
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
		
		//for PAllel
		else if(BT_Data=='K'){
		//	GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			pulse_width=2500;
			pulse_width1=2500;
			pulse_width2=500;
			pulse_width3=500;
			
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
		//	motorPWM=motorDIR;//stop motor1
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
		//	motorPWM_2=motorDIR_2;//stop motor2
			
			//stop motor
			motorDIR_3=GPIO_read(Dir_Pin_3);
		//	motorPWM_3=motorDIR_3;//stop motor1
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
		//	motorPWM_4=motorDIR_4;//stop motor2
			
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
			
			
		
			Change_flag=0;
		}
		
		//Rotation
		else if(BT_Data=='I'&&(Change_flag==1)){
			
			pulse_width=2000;
			pulse_width1=2000;
			pulse_width2=1000;
			pulse_width3=1000;
			
			GPIO_write(Dir_Pin_1,1);
			GPIO_write(Dir_Pin_2,0);
			GPIO_write(Dir_Pin_3,0);
			GPIO_write(Dir_Pin_4,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			//pulse_width=1500;
			//pulse_width1=1500;
			//stop motor
			motorDIR=GPIO_read(Dir_Pin_1);
			motorPWM=0.5;//rotation
					
			motorDIR_2=GPIO_read(Dir_Pin_2);
			motorPWM_2=0.5;//rotation
			
			motorDIR_3=GPIO_read(Dir_Pin_3);
			motorPWM_3=0.5;//rotation
					
			motorDIR_4=GPIO_read(Dir_Pin_4);
			motorPWM_4=0.5;//rotation

			Change_flag=1;
//			GPIO_write(Change_pin,1);
		}
		
		/*
		if (Mode_flag==1){
			switch(BT_Data){
				case 'Y':
					Ultra_flag=1;
				Parking_flag=0;
				break;
				
				case 'y':
					Ultra_flag=0;
				Parking_flag=0;
				break;
				
				case 'C':
					Ultra_flag2=1;
				break;
				
				case 'V':
					Parking_flag=1;
				break;
				
				case 'X':
					Parking_flag=0;
				Ultra_flag=0;
				Ultra_flag2=0;
				break;
				
				case 'Z':
					fire_flag=1;
				break;
			}
		}*/
		
	
				
				
		}
		
	}
}

//int state=0;//0->up, 1->down
uint32_t _count = 0;//to change 500msec
uint32_t _count1 = 0;//to change 500msec




void TIM5_IRQHandler(void){
	if(is_UIF(TIM5)){ 
		cnt++;//timer interrupt
		cnt1++;
		//Ultra_flag=0;
	//	printf("working111\r\n");
//		printf("mod: %d\r\n",Mode_flag);
	//		cnt1++;//for led count
		
//		Ultra_flag=GPIO_read(Ultra_pin1);
//		printf("ULTRA FLAG %d\r\n",Ultra_flag);
		
		
		
		if((cnt1>100)&&(Mode_flag==1)){
				Ultra_flag=GPIO_read(Ultra_pin1);
		Ultra_flag2=GPIO_read(Ultra_pin2);
			
		//	printf("ULTRA 1: %d  Ultra 2: %d\r\n",Ultra_flag,Ultra_flag2);
		Parking_check_flag=GPIO_read(Parking_pin);
	//		printf("PARKING CHECK %d\r\n",Parking_check_flag);
			
		switch(Parking_check_flag){
			case 1:
				Parking_flag=1;
	//		printf("parking flag on\r\n");
			break;
		}
		cnt1=0;
		if(value3<500){
			fire_flag=1;
		}
	}
		
		if(((cnt>10)&&(Mode_flag==1)&&(Ultra_flag==0)&&(Ultra_flag2==0)&&(Back_flag==0)&&(Change_flag==0))||((cnt>10)&&(Mode_flag==1)&&(fire_flag==1)&&(Ultra_flag==0)&&(Ultra_flag2==0)&&(Change_flag==0))||((cnt>10)&&(Mode_flag==1)&&(Go_fire_flag==1)&&(Ultra_flag==0)&&(Change_flag==0))){
			
		if((value1<1700)&&(value2<1700)){
		//	printf("STRA\r\n");
			motorPWM=1.0;//0.8 duty ratio motor1
			motorPWM_2=1.0;//0.8 duty ratio motor2
			motorPWM_3=1.0;//0.8 duty ratio motor1
			motorPWM_4=1.0;//0.8 duty ratio motor2
	//		STRval=0;
			//STRstate='0';//go straight
		}
		else if((value1>1700)&&(value2<1700)){
			//printf("LEFTg\r\n");
			motorPWM=1.0;//0.8 duty ratio motor1
			motorPWM_2=0.0;//0.8 duty ratio motor2
			motorPWM_3=0.0;//0.8 duty ratio motor1
			motorPWM_4=1.0;//0.8 duty ratio motor2
	//		STRval=-1;//left
		//	STRstate='1';//turn left
		}
		else if((value1<1700)&&(value2>1700)){
		//	printf("RIGHT\r\n");
			motorPWM=0.0;//0.8 duty ratio motor1
			motorPWM_2=1.0;//0.8 duty ratio motor2
			motorPWM_3=1.0;//0.8 duty ratio motor1
			motorPWM_4=0.0;//0.8 duty ratio motor2
	//		STRval=1;//right
		//	STRstate='1';//turn right
		}
		else if((value1>1700)&&(value2>1700)){
		//	printf("STRAIGHT\r\n");
			motorPWM=1.0;//0.8 duty ratio motor1
			motorPWM_2=1.0;//0.8 duty ratio motor2
			motorPWM_3=1.0;//0.8 duty ratio motor1
			motorPWM_4=1.0;//0.8 duty ratio motor2

		}
	cnt=0;//clear interrupt count
	}
		
	if((cnt>10)&&(Back_flag==1)){
		if((value1<1700)&&(value2<1700)){
				//	printf("STRA\r\n");
					motorPWM=0.7;//0.8 duty ratio motor1
					motorPWM_2=0.7;//0.8 duty ratio motor2
					motorPWM_3=0.7;//0.8 duty ratio motor1
					motorPWM_4=0.7;//0.8 duty ratio motor2
			//		STRval=0;
					//STRstate='0';//go straight
				}
		else if((value1>1700)&&(value2<1700)){
			//printf("LEFTg\r\n");
			motorPWM=0.7;//0.8 duty ratio motor1
			motorPWM_2=0.0;//0.8 duty ratio motor2
			motorPWM_3=0.0;//0.8 duty ratio motor1
			motorPWM_4=0.7;//0.8 duty ratio motor2
	//		STRval=-1;//left
		//	STRstate='1';//turn left
		}
		else if((value1<1700)&&(value2>1700)){
		//	printf("RIGHT\r\n");
			motorPWM=0.0;//0.8 duty ratio motor1
			motorPWM_2=0.7;//0.8 duty ratio motor2
			motorPWM_3=0.7;//0.8 duty ratio motor1
			motorPWM_4=0.0;//0.8 duty ratio motor2
	//		STRval=1;//right
		//	STRstate='1';//turn right
		}
		else if((value1>1700)&&(value2>1700)){
		//	printf("STRAIGHT\r\n");
			motorPWM=0.7;//0.8 duty ratio motor1
			motorPWM_2=0.7;//0.8 duty ratio motor2
			motorPWM_3=0.7;//0.8 duty ratio motor1
			motorPWM_4=0.7;//0.8 duty ratio motor2
		//	STRval=0;
		//	STRstate='0';//go straight
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
		value1 = JADC_read(1);//PB0
		value2 = JADC_read(2);//PB1
		value3 = JADC_read(3);//PC0
		clear_ADC_JEOC();//clear JEOC
	}
}

