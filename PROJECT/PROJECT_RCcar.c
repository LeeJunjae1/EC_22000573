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

volatile uint32_t moterDIR;
volatile float moterPWM=0.8; //PWM duty

volatile uint32_t moterDIR_2;
volatile float moterPWM_2=0.8; //PWM duty
volatile uint32_t Mode_flag=0;// check mode is auto or manual
uint8_t current_mode = 0;//to print current mode
uint8_t current_dir = 0;//to print current dir

uint32_t cnt=0;

void setup(void);

int main(void) {
	// Initialiization --------------------------------------------------------
	setup();
	printf("Hello Nucleo\r\n");
	USART_write(USART1,(uint8_t*) "Hello bluetooth\r\n",17);
	// Inifinite Loop ----------------------------------------------------------
	while (1){
		
		printf("MOD: %c DIR: %c\r\n",current_mode, current_dir);
		delay_ms(1000);
	}
			
}

// Initialiization 
void setup(void)
{
	RCC_PLL_init();

	UART2_init();//keyboard
	UART2_baud(BAUD_9600);
	
	UART1_init(); //bluetooth
	UART1_baud(BAUD_9600);
	
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
	
	TIM_UI_enable(TIM4);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 1);
	
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
                        printf("up key\r\n");
                        break;
                    case 'B': // down key
                        printf("down key\r\n");
                        break;
                    case 'C': // right key
                        printf("right key\r\n");
                        break;
                    case 'D': // left key
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
		
		if (btData=='L') {
       bReceive=1;  // Prepare to receive the next character
			moterPWM=0.5;// left, 0.5 duty ratio motor2
			moterPWM_2=0.8;// left, 0.8 duty ratio motor1
			printf("Turn Left\r\n\n");
        }
			else if((btData=='R'&&bReceive==1)||(btData=='R'&&bReceive==0)){
					moterPWM=0.8;// right, 0.8 duty ratio motor1
					moterPWM_2=0.5;// right, 0.5 duty ratio motor2
					printf("Turn Right\r\n\n");
				}
				else if((btData=='U'&&bReceive==1)||(btData=='U'&&bReceive==0)){
					moterPWM=0.8;//0.8 duty ratio motor1
					moterPWM_2=0.8;//0.8 duty ratio motor2
					printf("Go straight\r\n\n");
				}
				else if((btData=='S'&&bReceive==1)||(btData=='S'&&bReceive==0)){
					moterDIR=GPIO_read(Dir_Pin_1);
					moterPWM=moterDIR;//stop motor1
					
					moterDIR_2=GPIO_read(Dir_Pin_2);
					moterPWM_2=moterDIR_2;//stop motor2
					printf("Stop\r\n\n");
				}
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
        }
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

void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){ 
		cnt++;
		if(cnt>500){
		//PWM_duty(PWM_PIN, change_duty);
		printf("value1 = %d \r\n",value1);
		printf("value2 = %d \r\n",value2);
		printf("\r\n");
		if((value1<1500)&&(value2<1500)){
			printf("Go straight\r\n\n");
			moterPWM=0.8;//0.8 duty ratio motor1
			moterPWM_2=0.8;//0.8 duty ratio motor2
		}
		else if((value1>1500)&&(value2<1500)){
			printf("Go Left\r\n\n");
			moterPWM=0.5;// left, 0.5 duty ratio motor2
			moterPWM_2=0.8;// left, 0.8 duty ratio motor1
		}
		else if((value1<1500)&&(value2>1500)){
			printf("Go Right\r\n\n");
			moterPWM=0.8;// right, 0.8 duty ratio motor1
					moterPWM_2=0.5;// right, 0.5 duty ratio motor2
		}
		else if((value1>1500)&&(value2>1500)){
			printf("Go Straight\r\n\n");
			moterPWM=0.8;//0.8 duty ratio motor1
			moterPWM_2=0.8;//0.8 duty ratio motor2
		}
	cnt=0;
	}
		
		// clear by writing 0
		clear_UIF(TIM4); 		// Clear UI flag by writing 0              
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
