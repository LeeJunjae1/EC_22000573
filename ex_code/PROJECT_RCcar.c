/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-10-30 by JunjaeLee
* @brief   Embedded Controller: Project Line tracing
*
******************************************************************************
*/

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
#include "ecADC2.h"

#define END_CHAR 13
#define MAX_BUF 100

#define Moter_1 PA_0 //TIM2 Ch1
#define Moter_2 PA_1 //TIM2 Ch2
#define Dir_Pin_1 PC_2 //direction pin
#define Dir_Pin_2 PC_3 //direction pin
#define TRIG PA_6 //pwm
#define ECHO PB_6 //echo

uint8_t pcData = 0;//pcdata
uint8_t btData = 0;//bluetooth data
uint8_t buffer[MAX_BUF] = {0, };
int bReceive = 0;//check bt data received
int idx = 0;

//IR parameter//
uint32_t value1, value2;//value1: left ir sensor, value2: right ir sensor
PinName_t seqCHn[2] = {PB_0, PB_1}; //use to pin as adc


float duty=0.8;
float duty_2=0.8;

volatile uint32_t moterDIR; //motor1 direction
volatile float moterPWM=0.8; //motor1 PWM duty

volatile uint32_t moterDIR_2; //motor2 direction
volatile float moterPWM_2=0.8; //motor2 PWM duty
volatile uint32_t Mode_flag=0;// check mode is auto or manual
uint8_t current_mode = 0;//to print current mode
uint8_t current_dir = 0;//tmo print current dir

uint32_t cnt=0;//tim4 interuppt 
uint32_t cnt1=0;//to change led
float motorPWMState[4]={0.45f,0.60f,0.75f, 0.90f};// Straight speed
int PWMstate=0;//motor speed

int STRval=0; //STR value, negative: left, positive: right
float motorLeft[7]={0.4f,0.5f,0.6f,0.7f,0.8f,0.8f,0.8f};
float motorRight[7]={0.8f,0.8f,0.8f,0.7f,0.6f,0.5f,0.4f};

uint32_t ovf_cnt = 0;//count over count
float distance = 0;
float timeInterval = 0;
float time1 = 0;//start time
float time2 = 0;//end time
volatile uint32_t Ultra_flag=0;// to check object


void setup(void);

int main(void) {
	// Initialiization --------------------------------------------------------
	setup();
	printf("Hello Nucleo\r\n");
	USART_write(USART1,(uint8_t*) "Hello bluetooth\r\n",17);
	// Inifinite Loop ----------------------------------------------------------
	while (1){
		//printf("start \r\n");
		printf("MOD: %c DIR: %c STR: %d VEL: %d\r\n",current_mode, current_dir,STRval,PWMstate);
		printf("distance is %f\r\n\n",distance);
		delay_ms(1000);//delay 1sec
	}
			
}

// Initialiization 
void setup(void)
{
	RCC_PLL_init();

	UART2_init();//keyboard
	UART2_baud(BAUD_38400);
	
	UART1_init(); //bluetooth
	UART1_baud(BAUD_9600);
	
	SysTick_init(1000);			// SysTick Init
	
	GPIO_init(LED_PIN, OUTPUT);  //led pin
	
	//motor2
	PWM_init(Moter_1);// pwm settnig
	PWM_period_us(Moter_1,500);//0.5msec, 2kHz
	PWM_duty(Moter_1,duty);//change duty
	
	//motor2
	PWM_init(Moter_2);// pwm settnig
	PWM_period_us(Moter_2,500);//0.5msec, 2kHz
	PWM_duty(Moter_2,duty_2);//change duty
	
	//Direct pin
	GPIO_init(Dir_Pin_1, OUTPUT); //Calls RCC_GPIO_enable(), output
	GPIO_otype(Dir_Pin_1, 0);//push-pull
	
	//Direct pin
	GPIO_init(Dir_Pin_2, OUTPUT); //Calls RCC_GPIO_enable(), output
	GPIO_otype(Dir_Pin_2, 0);//push-pull
	GPIO_write(Dir_Pin_2,1);//high
	
	//TIM2
	//TIM_init(TIM2); //1msec
	TIM_UI_enable(TIM2);
	NVIC_EnableIRQ(TIM2_IRQn);	// TIM2's interrupt request enabled	
	//NVIC_SetPriority(TIM2_IRQn, 3);
	
	// ADC Init  Default: HW triggered by TIM3 counter @ 1msec
	//change tim3->tim4
	JADC_init(PB_0);
	JADC_init(PB_1);

	// ADC channel sequence setting
	JADC_sequence(seqCHn, 2);
	
	//TIM5 enable for ADC
	TIM_UI_enable(TIM5);
	NVIC_EnableIRQ(TIM5_IRQn);
	NVIC_SetPriority(TIM5_IRQn, 3);
	
	//INIT ultra sonic
	GPIO_otype(TRIG, 0);//push pull
	GPIO_pupd(TRIG,0);//NO pull-up pull-down
	GPIO_ospeed(TRIG,Fast_speed);//FAST SPEED
	
	// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG);			// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us
	
	// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO);    	// PB_6 as input caputre
	GPIO_pupd(ECHO,0);//NO pull-up pull-down
 	ICAP_counter_us(ECHO, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect

	
}

void USART1_IRQHandler(){         //USART1 INT 
	if(is_USART_RXNE(USART1)){
		btData = USART_read(USART1);//bluetooth data
		
		//check direction key
		if (btData == 0x1B) { // ESC key
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
										moterPWM=motorPWMState[PWMstate];
										moterPWM_2=motorPWMState[PWMstate];
										printf("current speed state is %d\r\n",PWMstate);
                        printf("up key\r\n");
                        break;
										
                    case 'B': // down key
											PWMstate--;
										if(PWMstate<0){
											PWMstate=0;
										}
										moterPWM=motorPWMState[PWMstate];
										moterPWM_2=motorPWMState[PWMstate];
										printf("when press down current speed is %d\r\n",PWMstate);
                        printf("down key\r\n");
                        break;
										
                    case 'C': // right key
											STRval++;
										if(STRval>3){
											STRval=3; //MAX 3
										}
										printf("current str is: %d\r\n",STRval);
										moterPWM=motorLeft[STRval+3];//change left motor speed
										moterPWM_2=motorRight[STRval+3];//change right motor speed
                        printf("right key\r\n");
                        break;
										
                    case 'D': // left key
										STRval--;
										if(STRval<-3){
											STRval=-3;//MIN -3
										}
										printf("WHEN PUSH left current str is: %d\r\n",STRval);
										moterPWM=motorLeft[STRval+3];//change left motor speed
										moterPWM_2=motorRight[STRval+3];//change right motor speed
										printf("left key\r\n"); 
                        break;
                    default:
                        break;
                }
            }
        } 
		else{
		
		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
		USART_write(USART1, &btData, 1);
		USART_write(USART1,(uint8_t*) "\r\n", 2);

		printf("NUCLEO got : %c (from BT)\r\n",btData);
		
		
			if((btData=='M')||(btData=='m')){
					current_mode='M';//manual mode
					GPIO_write(LED_PIN,1);//LED ON
					Mode_flag=0;
				}
				else if((btData=='A')||(btData=='a')){
					current_mode='A';//auto mode
					Mode_flag=1;
				}
				
			else if (((btData=='l')||(btData=='L')||(btData=='Q')||(btData=='q'))&&(Mode_flag==0)) {
       bReceive=1;  // Prepare to receive the next character
			moterPWM=0.5;// left, 0.5 duty ratio motor2
			moterPWM_2=0.8;// left, 0.8 duty ratio motor1
			printf("Turn Left\r\n\n");
        }
			else if(((btData=='R')||(btData=='r')||(btData=='E')||(btData=='e'))&&(Mode_flag==0)){
					moterPWM=0.8;// right, 0.8 duty ratio motor1
					moterPWM_2=0.5;// right, 0.5 duty ratio motor2
					printf("Turn Right\r\n\n");
				}
				else if(((btData=='F')||(btData=='f')||(btData=='W')||(btData=='w'))&&(Mode_flag==0)){
					
					current_dir='F';//current dirction is forward
					//Go straight
					GPIO_write(Dir_Pin_1,0);
					GPIO_write(Dir_Pin_2,1);
					
					
					//read motor dir
					moterDIR=GPIO_read(Dir_Pin_1);
					moterDIR_2=GPIO_read(Dir_Pin_2);
					
					moterPWM=1.0;//0.8 duty ratio motor1
					moterPWM_2=1.0;//0.8 duty ratio motor2
					printf("Go straight\r\n\n");
				}
				else if(((btData=='b')||(btData=='B')||(btData=='W')||(btData=='w'))&&(Mode_flag==0)){
					
					current_dir='B';//current dirction is forward
					//Back
					GPIO_write(Dir_Pin_1,1);//change direction
					GPIO_write(Dir_Pin_2,0);//change directino
					
					
					//read motor dir
					moterDIR=GPIO_read(Dir_Pin_1);
					moterDIR_2=GPIO_read(Dir_Pin_2);
					
					moterPWM=1.0;//1.0 duty ratio motor1
					moterPWM_2=1.0;//1.0 duty ratio motor2
					printf("Moving Backward\r\n\n");
				}
				else if(((btData=='S')||(btData=='s'))&&(Mode_flag==0)){
					moterDIR=GPIO_read(Dir_Pin_1);
					moterPWM=moterDIR;//stop motor1
					
					moterDIR_2=GPIO_read(Dir_Pin_2);
					moterPWM_2=moterDIR_2;//stop motor2
					printf("Stop\r\n\n");
				}
				
				
				/*
				else if (bReceive==1) {
         if (btData=='0') {
            GPIO_write(LED_PIN, 0);  // Turn OFF LED
					 printf("LED OFF\r\n\n");
             //USART_write(USART1, (uint8_t*) "LED OFF\r\n", 10);
					 
            } else if (btData=='1') {
              GPIO_write(LED_PIN, 1);  // Turn ON LED
							printf("LED ON\r\n\n");
             // USART_write(USART1, (uint8_t*) "LED ON\r\n", 9);
            }
            bReceive = 0;  // Reset for next command
        }*/
			
		}
		
	}
}
void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){ 
		//PWM_duty(PWM_PIN, change_duty);
		
		duty=fabs(moterDIR-moterPWM);//change speeed
		PWM_duty(Moter_1,duty);//change duty
		
		duty_2=fabs(moterDIR_2-moterPWM_2);//change speeed
		PWM_duty(Moter_2,duty_2);//change duty
		
		// clear by writing 0
		clear_UIF(TIM2); 		// Clear UI flag by writing 0              
	}
}
/*
void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                     // Update interrupt
		ovf_cnt++;													// overflow count
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
			Ultra_flag=1;
			printf("detect object\r\n\n");
			
			moterDIR=GPIO_read(Dir_Pin_1);
			moterPWM=moterDIR;//stop motor1
					
			moterDIR_2=GPIO_read(Dir_Pin_2);
			moterPWM_2=moterDIR_2;//stop motor2
			printf("Stop by ultrasonic sensor\r\n\n");
		}
		else{
			Ultra_flag=0;
		}
		clear_CCIF(TIM4,2);								  // clear capture/compare interrupt flag 
	}
}*/

void TIM5_IRQHandler(void){
	if(is_UIF(TIM5)){ 
		cnt++;
		cnt1++;
		if((cnt>10)&&(Mode_flag==1)){
			//for interrupt 10msec
		//PWM_duty(PWM_PIN, change_duty);
			/*
		printf("value1 = %d \r\n",value1);
		printf("value2 = %d \r\n",value2);
		printf("\r\n");*/
		if((value1<1500)&&(value2<1500)&&(Ultra_flag==0)){
			printf("Go straight\r\n\n");
			moterPWM=0.9;//0.8 duty ratio motor1
			moterPWM_2=0.9;//0.8 duty ratio motor2
		}
		else if((value1>1500)&&(value2<1500)&&(Ultra_flag==0)){
			printf("Go Left\r\n\n");
			moterPWM=0.5;// left, 0.5 duty ratio motor2
			moterPWM_2=0.8;// left, 0.8 duty ratio motor1
		}
		else if((value1<1500)&&(value2>1500)&&(Ultra_flag==0)){
			printf("Go Right\r\n\n");
			moterPWM=0.8;// right, 0.8 duty ratio motor1
					moterPWM_2=0.5;// right, 0.5 duty ratio motor2
		}
		else if((value1>1500)&&(value2>1500)&&(Ultra_flag==0)){
			printf("Go Straight\r\n\n");
			moterPWM=0.9;//0.8 duty ratio motor1
			moterPWM_2=0.9;//0.8 duty ratio motor2
		}
	cnt=0;
	}
		if(cnt1>500&&(Mode_flag==1)){
			LED_toggle();
			cnt1=0;
		}
		
		// clear by writing 0
		clear_UIF(TIM5); 		// Clear UI flag by writing 0              
	}
}


void USART2_IRQHandler(){         //USART2 INT 
 if (is_USART_RXNE(USART2)) {
        pcData = USART_read(USART2);
        
        if (pcData == 0x1B) { // ESC key
            uint8_t nextChar1 = USART_read(USART2);
            if (nextChar1 == '[') {
                uint8_t nextChar2 = USART_read(USART2);
                switch (nextChar2) {
                    case 'A': // up key
                        printf("Up key in usart2\r\n");
                        break;
                    case 'B': // down key
                        printf("Down key in usart2\r\n");
                        break;
                    case 'C': // right key
                        printf("Right key in usart2\r\n");
                        break;
                    case 'D': // left key
                        printf("Left key in usart2\r\n"); 
                        break;
                    default:
                        break;
                }
            }
        } else {
            USART_write(USART1, &pcData, 1);
            printf("Received: %c\r\n", pcData);
            if (pcData == END_CHAR) {
                printf("\r\n");
            }
        }
    }
}

void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
	//printf("call function\r\n");
	if(is_ADC_JEOC()){		// after finishing sequence
		value1 = JADC_read(1);
		value2 = JADC_read(2);
		clear_ADC_JEOC();
	}
}