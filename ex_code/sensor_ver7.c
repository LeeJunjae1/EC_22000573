/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-10-30 by JunjaeLee
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


//static volatile uint8_t PC_Data = 0;
//static volatile uint8_t BT_Data = 0;
//uint8_t PC_string[]="Loop:\r\n";

//uint8_t a;
//uint8_t b;
//uint8_t c;
//uint8_t d;
//char BT_string[15]={0};//to print sentence
//char BT_string1[15]={0};//to print sentence

/*
#define Servo_pin PB_10 //servo   RIGHT
//#define Servo_pin1 PB_10    //servo LEFT
#define Motor_1 PA_1 //TIM2 Ch1 Left motor
#define Motor_2 PA_0 //TIM2 Ch2 Right motor
#define Dir_Pin_1 PC_3 //direction pin
#define Dir_Pin_2 PC_2 //direction pin*/

#define TRIG1 PA_8 //trig1
#define ECHO1 PB_6 //echo1
#define TRIG2 PA_8 //trig2
#define ECHO2 PB_8 //echo2

#define TRIG3 PA_8 //trig3
#define ECHO3 PA_6 //echo3
#define TRIG4 PA_8 //trig4
#define ECHO4 PC_8 //echo4

#define Ultra_pin1 PA_5 //ultra sensor1
#define Ultra_pin2 PB_4 //ultra sensor2
#define Parking_pin PB_5 //parking flag 
#define Parking_finish_pin PB_14 //manual mode->0

//unsigned int pulse_width=500;//calculate pulse width;//calculate pulse width;//pulse width
//unsigned int pulse_width1=2500;//calculate pulse width;//calculate pulse width;//pulse width

/*
//motor duty
float duty=0.5;
float duty_2=0.5;

volatile uint32_t motorDIR; //motor1 direction
volatile float motorPWM=0.5; //motor1 PWM duty

volatile uint32_t motorDIR_2; //motor2 direction
volatile float motorPWM_2=0.5; //motor2 PWM duty
*/


volatile uint32_t Mode_flag=0;// check mode is auto or manual
uint32_t ovf_cnt = 0;//count over count1
uint32_t ovf_cnt2 = 0;//count over count2
float distance = 0;//forward distance
float distance2 = 0;//back distance

float timeInterval = 0;//forward ultra sensor time interval
float timeInterval2 = 0;//backward ultra sensor time interval
float time1 = 0;//start time1
float time2 = 0;//end time1

float time1_2 = 0;//start time2
float time2_2 = 0;//end time2

float time1_3 = 0;//start time3
float time2_3 = 0;//end time3

float time1_4 = 0;//start time4
float time2_4 = 0;//end time4

uint32_t ovf_cnt3 = 0;//count over count3
uint32_t ovf_cnt4 = 0;//count over count4
float distance3 = 0;//side distance
float distance4 = 0;//side distance

float timeInterval3 = 0;//
float timeInterval4 = 0;

float prev1 = 0;
float prev2 = 0;
float prev3 = 0;

volatile uint32_t Ultra_flag=0;// to check object
volatile uint32_t Ultra_flag2=0;// to check object
volatile uint32_t Ultra_flag3=0;// to check object
volatile uint32_t Ultra_flag4=0;// to check object

//volatile uint32_t Mode_flag=0;// check mode is auto or manual
volatile uint32_t Change_flag=0;// to check object
volatile uint32_t auto_state=0;// to check auto state
volatile uint32_t Parking_flag=0;// to check object
volatile uint32_t finish_flag=0;// to check object

//IR parameter//
uint32_t value1, value2;//value1: left ir sensor, value2: right ir sensor
PinName_t seqCHn[2] = {PB_0, PB_1}; //use to pin as jadc

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
	
	GPIO_init(Ultra_pin1, OUTPUT);  //led pin
	GPIO_init(Ultra_pin2, OUTPUT);  //led pin
	GPIO_init(Parking_pin, OUTPUT);  //led pin
	
	GPIO_write(Ultra_pin1,0);
	GPIO_write(Ultra_pin2,0);
	GPIO_write(Parking_pin,0);
	
	//auto mode
	GPIO_init(Parking_finish_pin, INPUT);  //led pin
//	GPIO_write(Mode_pin,1);//mode pin
	

	
	
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



	//INIT ultra sonic
	GPIO_otype(TRIG2, 0);//push pull
	GPIO_pupd(TRIG2,0);//NO pull-up pull-down
	GPIO_ospeed(TRIG2,Fast_speed);//FAST SPEED

	PWM_init(TRIG2);			// PA_6: Ultrasonic trig pulse
//	PWM_period_ms(TRIG2, 1);    // PWM of 50ms period. Use period_us()
	PWM_period_us(TRIG2, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG2, 10);   // PWM pulse width of 10us
	
	//Input Capture 2 
	ICAP_init(ECHO2);    	// PB_6 as input caputre
	GPIO_pupd(ECHO2,0);//NO pull-up pull-down
 	ICAP_counter_us(ECHO2, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO2, 3, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO2, 4, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect

	//Input Capture 3
// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG3);			// PA_6: Ultrasonic trig pulse
	//PWM_period_ms(TRIG1, 1);    // PWM of 50ms period. Use period_us()
	PWM_period_us(TRIG3, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG3, 10);   // PWM pulse width of 10us

	// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO3);    	// PB_6 as input caputre
	GPIO_pupd(ECHO3,0);//NO pull-up pull-down
 	ICAP_counter_us(ECHO3, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO3, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO3, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect



	//INIT ultra sonic
	GPIO_otype(TRIG4, 0);//push pull
	GPIO_pupd(TRIG4,0);//NO pull-up pull-down
	GPIO_ospeed(TRIG4,Fast_speed);//FAST SPEED

	PWM_init(TRIG4);			// PA_6: Ultrasonic trig pulse
//	PWM_period_ms(TRIG2, 1);    // PWM of 50ms period. Use period_us()
	PWM_period_us(TRIG4, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG4, 10);   // PWM pulse width of 10us
	
	//Input Capture 2 
	ICAP_init(ECHO4);    	// PB_6 as input caputre
	GPIO_pupd(ECHO4,0);//NO pull-up pull-down
 	ICAP_counter_us(ECHO4, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO4, 3, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO4, 4, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect


}

int main(void){	
	setup();
	printf("Hello Nucleo\r\n");
	USART_write(USART1,(uint8_t*) "Hello bluetooth\r\n",17);
	while(1){
		// USART Receive: Use Interrupt only
		// USART Transmit:  Interrupt or Polling
		//USART2_write(PC_string, 7);
		printf("distance1: %f cm\r\n", distance);//display distance
		printf("distance2: %f cm\r\n\n", distance2);//display distance
		printf("distance3: %f cm\r\n", distance3);//display distance
		printf("distance4: %f cm\r\n\n", distance4);//display distance
		
		printf("PARKING flag : %d\r\n",Parking_flag);

	//	printf("val1: %d \r\n", value1);//display distance
//		printf("val2: %d \r\n\n", value2);//display distance

		//		printf("time2: \r\n");
	//	printf("ovf1: %d dis1: %f tim1: %f, tim2: %f\r\n",ovf_cnt,distance,time1,time2);
	//	printf("ovf: %d, dis2: %f tim1: %f tim2: %f\r\n\n",ovf_cnt2,distance2,time1_2, time2_2);
		delay_ms(500);        
	}
}

/*
void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		PC_Data = USART2_read();		// RX from UART2 (PC)
	//	USART_write(USART1,(uint8_t*) "PC sent : ", 10);
		USART_write(USART2,(uint8_t*) "usart2: ",8);
		USART2_write((uint8_t*)&PC_Data,1);		// TX to USART2	 (PC)	 Echo of keyboard typing		
	USART_write(USART2,(uint8_t*) "\r\n",2);
		if(PC_Data=='L'){
			GPIO_write(LED_PIN,0);
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
		else if(PC_Data=='H'){
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
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
		else if(PC_Data=='A'){
			GPIO_write(LED_PIN,1);
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
		else if(PC_Data=='B'){
			//auto mode
			Mode_flag=1;
	//		pulse_width=500;
	//		pulse_width1=2500;
			Change_flag=0;
		}
		else if(PC_Data=='T'){
			//auto mode
			finish_flag=1;
		}
	}
}*/

/*

void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
		BT_Data = USART1_read();		// RX from UART1 (BT)		
		//printf("RX: %c \r\n",BT_Data); // TX to USART2(PC)
		USART_write(USART1,(uint8_t*) "RX: ",4);
		USART_write(USART1,(uint8_t*)&BT_Data,1);
		USART_write(USART1,(uint8_t*) "\r\n",2);
		USART_write(USART2,(uint8_t*)&BT_Data,1);
		if(BT_Data=='L'){
			GPIO_write(LED_PIN,0);
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
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
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
		else if(BT_Data=='A'){
			GPIO_write(LED_PIN,1);
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
		else if(BT_Data=='B'){
			//auto mode
			Mode_flag=1;
			pulse_width=500;
			pulse_width1=2500;
			Change_flag=0;
		}

		
	}
}
*/

int state=0;//0->up, 1->down
uint32_t _count = 0;//to change 500msec
uint32_t degree = 0;//to change rotation
uint32_t rotation = 1500;//to change rotation degree 45 degree or 90 degree
uint32_t set_degree = 45;//45 degree or 90 degree


/*
void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){                     // Update interrupt
		ovf_cnt++;													// overflow count
		clear_UIF(TIM3);  							    // clear update interrupt flag
	}
	if(is_CCIF(TIM3, 3)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM3,3);									// Capture TimeStart
		clear_CCIF(TIM3, 3);                // clear capture/compare interrupt flag 
	}								                      
	else if(TIM3,4){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2 = ICAP_capture(TIM3,4);									// Capture TimeEnd
		timeInterval = ((time2-time1)+ovf_cnt*((TIM3->ARR)+1))/100.0; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		distance = (float) timeInterval * 340.0 / 2.0 /10.0; 	// [mm] -> [cm]
		ovf_cnt = 0;                        // overflow reset
		clear_CCIF(TIM3,4);								  // clear capture/compare interrupt flag 
	}
}*/

void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){                     // Update interrupt
		ovf_cnt3++;													// overflow count
		ovf_cnt4++;													// overflow count
		clear_UIF(TIM3);  							    // clear update interrupt flag
	}
	Mode_flag=1;
	
	if(is_CCIF(TIM3, 1)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1_3 = ICAP_capture(TIM3,1);									// Capture TimeStart
		clear_CCIF(TIM3, 1);                // clear capture/compare interrupt flag 
	}								                      
	else if(TIM3,2){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2_3 = ICAP_capture(TIM3,2);									// Capture TimeEnd
		timeInterval3 = ((time2_3-time1_3)+ovf_cnt3*((TIM3->ARR)+1))/100.0; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		distance3 = (float) timeInterval3 * 340.0 / 2.0 /10.0; 	// [mm] -> [cm]
		
		ovf_cnt3 = 0;                        // overflow reset
		
		
		// delete mode flag
		if(((distance3<80.0)&&(distance3>38.0))||((prev3<80.0)&&(prev2>38.0))){
			Ultra_flag3=1;// detect object
			
			//stop motor
		//	motorDIR=GPIO_read(Dir_Pin_1);
		//	motorPWM=motorDIR;//stop motor1
					
	//		motorDIR_2=GPIO_read(Dir_Pin_2);
	//		motorPWM_2=motorDIR_2;//stop motor2
		}
		else{
			Ultra_flag3=0;//no object
		}
		prev3=distance3;
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
		
		if(((distance4<80.0)&&(distance4>38.0))){
			Ultra_flag4=1;// detect object
			if((Ultra_flag3==1)&&(Ultra_flag4==1)){
				Parking_flag=1;
				
			}
			
			//stop motor
		//	motorDIR=GPIO_read(Dir_Pin_1);
		//	motorPWM=motorDIR;//stop motor1
					
	//		motorDIR_2=GPIO_read(Dir_Pin_2);
		//	motorPWM_2=motorDIR_2;//stop motor2
		}
		else{
			Ultra_flag4=0;//no object
		}
		clear_CCIF(TIM3,4);								  // clear capture/compare interrupt flag 
	}
	if(Parking_flag==1){
		//parking mode
		GPIO_write(Parking_pin,1);
	}
	
	//parking finish
	if(finish_flag==1){
		GPIO_write(Parking_pin,0);
		finish_flag=0;
		Parking_flag=0;
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
		
		if(((distance<6.0)&&(distance>2.0))||((prev1<6.0)&&(prev1>1.5))){
			Ultra_flag=1;// detect object
			GPIO_write(Ultra_pin1,1);
			
			//stop motor
	//		motorDIR=GPIO_read(Dir_Pin_1);
	//		motorPWM=motorDIR;//stop motor1
					
	//		motorDIR_2=GPIO_read(Dir_Pin_2);
	//		motorPWM_2=motorDIR_2;//stop motor2
		}
		else{
			Ultra_flag=0;//no object
			GPIO_write(Ultra_pin1,0);
		}
		prev1=distance;
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
		
		if(((distance2<6.0)&&(distance2>1.5))||((prev2<6.0)&&(prev2>1.5))){
			Ultra_flag2=1;// detect object
			GPIO_write(Ultra_pin2,1);
			
			//stop motor
		//	motorDIR=GPIO_read(Dir_Pin_1);
		//	motorPWM=motorDIR;//stop motor1
					
		//	motorDIR_2=GPIO_read(Dir_Pin_2);
		//	motorPWM_2=motorDIR_2;//stop motor2
		}
		else{
			Ultra_flag2=0;//no object
			GPIO_write(Ultra_pin2,0);
		}
		prev2=distance2;
		clear_CCIF(TIM4,4);								  // clear capture/compare interrupt flag 
	}
}


