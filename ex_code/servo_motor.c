//#include "ecSTM32F4v2.h"
#include "stm32f411xe.h"
#include "math.h"
//#include "ecSTM32F4v2.h" //if I change my header file, I have to rebuild it. So I don't use it. I check MCU normally running, when I use this header file.

#include "ecPinNames.h"
#include "ecRCC2.h"
#include "ecGPIO2.h"
#include "ecEXTI2.h"
#include "ecSysTick2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "ecICAP2.h"
//#include "ecUART2_simple.h"
#include "ecUART2.h" 


static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
uint8_t PC_string[]="Loop:\r\n";

uint8_t a;
uint8_t b;
uint8_t c;
uint8_t d;
char BT_string[15]={0};//to print sentence
char BT_string1[15]={0};//to print sentence

#define Servo_pin PA_1 //servo   
#define Servo_pin1 PA_0    //servo
unsigned int pulse_width=500;//calculate pulse width;//calculate pulse width;//pulse width
unsigned int pulse_width1=2500;//calculate pulse width;//calculate pulse width;//pulse width

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
	
	//direction
	GPIO_init(PC_0, OUTPUT);  //led pin
	GPIO_init(PC_1, OUTPUT);  //led pin
	
	GPIO_init(PC_4, OUTPUT);  //led pin
	
	GPIO_init(PC_2, INPUT);  //led pin
	GPIO_init(PC_3, INPUT);  //led pin
	GPIO_pupd(PC_2, EC_PU);
	GPIO_pupd(PC_3, EC_PU);
	GPIO_write(PC_4,1);
	GPIO_write(PA_5,1);
	GPIO_write(PC_1,1);
	
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
	//Timer interrupt of 500msec
	TIM_UI_init(TIM3, 500);			// TIM3 Update-Event Interrupt every 500 msec
	TIM_UI_enable(TIM3); //TIM3 enable

}

int main(void){	
	setup();
	printf("Hello Nucleo\r\n");
	USART_write(USART1,(uint8_t*) "Hello bluetooth\r\n",17);
	while(1){
		// USART Receive: Use Interrupt only
		// USART Transmit:  Interrupt or Polling
		//USART2_write(PC_string, 7);
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
		if('L'==PC_Data){
			GPIO_write(LED_PIN,0);
		}
		else if('H'==PC_Data){
			GPIO_write(LED_PIN,1);
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
		if(BT_Data=='L'){
			GPIO_write(LED_PIN,0);
			//GPIO_write(PC_0,0);
			//GPIO_write(PC_1,0);
			pulse_width=1000;
			pulse_width1=2000;
			
			
		}
		else if(BT_Data=='H'){
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			pulse_width=500;
			pulse_width1=2500;
			

			
		}
		
		//for change servo motor
		else if(BT_Data=='A'){
			GPIO_write(LED_PIN,1);
		//	GPIO_write(PC_0,0);
		//	GPIO_write(PC_1,1);
			//90 degree
			pulse_width=1500;
			pulse_width1=1500;

			
		}
		
		/*
		b=GPIO_read(PC_0);
		a=GPIO_read(PC_1);
		c=GPIO_read(PC_2);
		d=GPIO_read(PC_3);
		sprintf(BT_string,"a: %d b: %d\r\n",a,b);
		USART1_write((uint8_t*)BT_string, 15);
		sprintf(BT_string1,"c: %d d: %d\r\n",c,d);
		USART1_write((uint8_t*)BT_string1, 15);
		*/
		
		
		/*
		USART_write(USART1,(uint8_t*) "a: ",3);
		USART_write(USART1,a,1);
		
		USART_write(USART1,(uint8_t*) " b: ",4);
		USART_write(USART1,b,1);
		USART_write(USART1,(uint8_t*) "\r\n",2);*/
		
		/*
			if('L'==PC_Data){
			GPIO_write(LED_PIN,0);
		}
		else if('H'==PC_Data){
			GPIO_write(LED_PIN,1);
		}*/
		
	}
}

int state=0;//0->up, 1->down
uint32_t _count = 0;//to change 500msec
uint32_t degree = 0;//to change rotation
uint32_t rotation = 1500;//to change rotation degree 45 degree or 90 degree
uint32_t set_degree = 45;//45 degree or 90 degree



void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)

			//if(state==0){//up state
			//	pulse_width=500;
				//pulse_width=500+degree/set_degree*(rotation-500);//calculate pulse width
			  PWM_pulsewidth_us(Servo_pin,pulse_width);//chagne pulse width
				PWM_pulsewidth_us(Servo_pin1,pulse_width1);//chagne pulse width
				//degree+=10;//plus 10 degree
			//	if(degree>=set_degree){
				//	state=1;//go down state
				//	degree=set_degree;// degree is 180 degree
				//}
		//}
			/*
			else{//down state
			//pulse_width=500+degree/18.*(2500-500);//calculate pulse width
			PWM_pulsewidth_us(Moter_pin,pulse_width);//chagne pulse width
			degree--;//minus 10 degree
			if(degree<=0){
				state=0;//go up state
				degree=0;//degree is 0 degree
			
		}*/
		clear_UIF(TIM3); 		// Clear UI flag by writing 
	}
}