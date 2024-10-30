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

//IR parameter//
uint32_t value1, value2;
uint32_t cnt=0;
uint32_t cnt1=0;
PinName_t seqCHn[2] = {PB_0, PB_1};

void setup(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		printf("value1 = %d \r\n",value1);
		printf("value2 = %d \r\n",value2);
		printf("\r\n");
		if((value1<1500)&&(value2<1500)){
			printf("Go straight\r\n\n");
		}
		else if((value1>1500)&&(value2<1500)){
			printf("Go Left\r\n\n");
		}
		else if((value1<1500)&&(value2>1500)){
			printf("Go Right\r\n\n");
		}
		else if((value1>1500)&&(value2>1500)){
			printf("Go Straight\r\n\n");
		}
		delay_ms(1000);
	}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                 // System Clock = 84MHz
	UART2_init();			// UART2 Init
	SysTick_init(1000);			// SysTick Init
	
	// ADC Init  Default: HW triggered by TIM3 counter @ 1msec
	//change tim3->tim2
	JADC_init(PB_0);
	JADC_init(PB_1);

	// ADC channel sequence setting
	JADC_sequence(seqCHn, 2);
}


void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
	//printf("call function\r\n");
	if(is_ADC_JEOC()){		// after finishing sequence
	//	printf("read 1\r\n");
		value1 = JADC_read(1);
	//	printf("read 2\r\n");
		value2 = JADC_read(2);
		//printf("val=%d val2=%d\r\n",value1,value2);
		clear_ADC_JEOC();
	}
}