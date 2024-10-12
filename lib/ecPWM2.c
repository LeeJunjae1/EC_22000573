/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-10-02 by JunjaeLee  	
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecPWM2.h"
#include "math.h"

/* PWM Configuration using PinName_t Structure */

/* PWM initialization */
// Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
void PWM_init(PinName_t pinName){

// 0. Match TIMx from  Port and Pin 	
	GPIO_TypeDef *port;
	unsigned int pin;
	ecPinmap(pinName, &port, &pin);
	TIM_TypeDef *TIMx;
	int chN;
	PWM_pinmap(pinName, &TIMx, &chN);
	
	
// 1. Initialize GPIO port and pin as AF	
	GPIO_init(pinName, AF);  // AF=2
	GPIO_otype(pinName, 0);//push-pull
	GPIO_pupd(pinName,0);//no pupd
	GPIO_ospeed(pinName,High_speed);//high speed

	
	
// 2. Configure GPIO AFR by Pin num.	
	//  AFR[0] for pin: 0~7,     AFR[1] for pin 8~15
	//  AFR=1 for TIM1,TIM2	AFR=2 for TIM3 etc			
	unsigned int AFR_val;
	
	if(port == GPIOA) {
      switch(pin){
         case 0 : AFR_val=0b0001; break; //tim2
         case 1 : AFR_val=0b0001; break;//tim2
         case 5 : AFR_val=0b0001; break;//tim2
         case 6 : AFR_val=0b0010; break;//tim3
         //case 7: AFR_val=0b0001; break;//tim1
         case 8 : AFR_val=0b0001; break;//tim1
         case 9 : AFR_val=0b0001; break;//tim1
         case 10: AFR_val=0b0001; break;//tim1
         case 15: AFR_val=0b0001; break;//tim2
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: AFR_val=0b0001; break; //tim1
         //case 1: AFR_val=0b0001; break;//tim1
         case 3 : AFR_val=0b0001; break;//tim2
         case 4 : AFR_val=0b0010; break;//tim3
         case 5 : AFR_val=0b0010; break;//tim3
         case 6 : AFR_val=0b0010; break;//tim4
         case 7 : AFR_val=0b0010; break;//tim4
         case 8 : AFR_val=0b0010; break;//tim4
         case 9 : AFR_val=0b0010; break;//tim4
         case 10: AFR_val=0b0001; break;//tim2     
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : AFR_val=0b0010; break;//tim3
         case 7 : AFR_val=0b0010; break;//tim3
         case 8 : AFR_val=0b0010; break;//tim3
         case 9 : AFR_val=0b0010; break;//tim3
         default: break;
      }
   }
	
if (pin<8){
	port->AFR[0]	 &= ~(0xF<<(pin*4));//clear
  port->AFR[0]	 |= (AFR_val<<(pin*4));//SET AFR VALUE
}
else if (pin<16){
	port->AFR[1]	 &= ~(0xF<<((pin-8)*4));//clear
  port->AFR[1]	 |= (AFR_val<<((pin-8)*4));//SET AFR VALUE
}

	
	
// 3. Initialize Timer 
	TIM_init(TIMx);	// with default msec=1msec value.		
	TIM_period(TIMx, 1);
	TIMx->CR1 &= ~TIM_CR1_CEN;
	
// 3-2. Direction of Counter
	//YOUR CODE GOES HERE
	TIMx->CR1 &= ~TIM_CR1_DIR;  //up counter~(1<<4)                        // Counting direction: 0 = up-counting, 1 = down-counting
			
	
// 4. Configure Timer Output mode as PWM
	uint32_t ccVal = ((TIMx->ARR)+1)*0.5;  // default value  CC=ARR/2
	if(chN == 1){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear ouput compare mode bits for channel 1
		//TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // OC1M = 110 for PWM Mode 1 output on ch1. #define TIM_CCMR1_OC1M_1          (0x2UL << TIM_CCMR1_OC1M_Pos)
		TIMx->CCMR1 |= (0b110<<4);
		TIMx->CCMR1	|= TIM_CCMR1_OC1PE;                     // Output 1 preload enable (make CCR1 value changable)
		TIMx->CCR1  = ccVal; 																// Output Compare Register for channel 1 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC1P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC1E;												// Enable output for ch1
	}
	else if(chN == 2){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;                     // Clear ouput compare mode bits for channel 2
		//TIMx->CCMR1 |= (0b110<<12) | (0b111<<12); // OC1M = 110 for PWM Mode 1 output on ch2. #define TIM_CCMR1_OC1M_1          (0x2UL << TIM_CCMR1_OC1M_Pos)
		//0b110 high->low, ob111 low->high
		TIMx->CCMR1 |= (0b110<<12);
		TIMx->CCMR1	|= 1<<11;                     // Output 1 preload enable (make CCR1 value changable)
		TIMx->CCR2  = ccVal; 																// Output Compare Register for channel 2 (default duty ratio = 50%)		
		TIMx->CCER &= ~(1<<5);                       // select output polarity: active high	
		TIMx->CCER  |= (1<<4);																// Enable output for ch2
	}
	else if(chN == 3){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;                     // Clear ouput compare mode bits for channel 3
		//TIMx->CCMR2 |= (0b110<<4) | (0b111<<4);															// OC1M = 110 for PWM Mode 1 output on ch3
		TIMx->CCMR2 |= (0b110<<4);
		TIMx->CCMR2	|= 1<<3;                					// Output 1 preload enable (make CCR3 value changable)	
		TIMx->CCR3  = ccVal;															// Output Compare Register for channel 3 (default duty ratio = 50%)		
		TIMx->CCER &= ~(1<<9);                     				// select output polarity: active high	
		TIMx->CCER  |= (1<<8);														// Enable output for ch3
	}
	else if(chN == 4){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC4M; 								// Clear ouput compare mode bits for channel 4
		TIMx->CCMR2 |= (0b110<<12);											// OC1M = 110 for PWM Mode 1 output on ch4
		//TIMx->CCMR2 |= (0b110<<12) | (0b111<<12);		
		TIMx->CCMR2	|= 1<<11;														// Output 1 preload enable (make CCR4 value changable)
		TIMx->CCR4  = ccVal;														// Output Compare Register for channel 4 (default duty ratio = 50%)	
		TIMx->CCER &= ~(1<<13); 												// select output polarity: active high	
		TIMx->CCER  |= (1<<12);													// Enable output for ch4
	}	
	
	
// 5. Enable Timer Counter
	// For TIM1 ONLY
	if(TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;					// Main output enable (MOE): 0 = Disable, 1 = Enable	
	// Enable timers
	TIMx->CR1  |= TIM_CR1_CEN;  													// Enable counter
	
}

/* PWM PERIOD SETUP */
// allowable range for msec:  1~2,000
void PWM_period_ms(PinName_t pinName,  uint32_t msec){
	
// 0. Match TIMx from  Port and Pin 	
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(pinName, &port, &pin);	
	TIM_TypeDef *TIMx;
	int chN;		
	PWM_pinmap(pinName, &TIMx, &chN);
	
	
// 1. Set Counter Period in msec
	TIM_period_ms(TIMx, msec);
	
}


// allowable range for msec:  1~2,000
void PWM_period(PinName_t pinName,  uint32_t msec){
	PWM_period_ms(pinName,  msec);
}


// allowable range for usec:  1~1,000
void PWM_period_us(PinName_t pinName,  uint32_t usec){

// 0. Match TIMx from  Port and Pin 	
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(pinName, &port, &pin);	
	TIM_TypeDef *TIMx;
	int chN;		
	PWM_pinmap(pinName, &TIMx, &chN);
	

// 1. Set Counter Period in usec
	TIM_period_us(TIMx,usec);
	
}

/* DUTY RATIO SETUP */
// High Pulse width in msec
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms){
// 0. Match TIMx from  Port and Pin 	
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(pinName, &port, &pin);	
	TIM_TypeDef *TIMx;
	int chN;		
	PWM_pinmap(pinName, &TIMx, &chN);
	

// 1. Declaration System Frequency and Prescaler
	uint32_t fsys = 0;
	uint32_t psc = TIMx->PSC;

	
// 2. Check System CLK: PLL or HSI
	if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL)  		fsys = 84000;  // for msec 84MHz/1000 [msec]
	else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI) fsys = 16000;

	
// 3. Configure prescaler PSC
	float fclk = fsys/(psc+1);					// fclk=fsys/(psc+1); 84000/840=100
	uint32_t value = pulse_width_ms *fclk - 1;					// pulse_width_ms *fclk - 1;

	switch(chN){
		//change ccr value
		case 1: TIMx->CCR1 = value; break;
		case 2: TIMx->CCR2 = value; break;
		case 3: TIMx->CCR3 = value; break;
		case 4: TIMx->CCR4 = value; break;
		default: break;
	}
}

// High Pulse width in msec
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms){
	PWM_pulsewidth(pinName, pulse_width_ms);
}
	
// High Pulse width in usec
void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us){
// 0. Match TIMx from  Port and Pin 	
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(pinName, &port, &pin);	
	TIM_TypeDef *TIMx;
	int chN;		
	PWM_pinmap(pinName, &TIMx, &chN);
	
// 1. Declaration system frequency and prescaler
	uint32_t fsys = 0;
	uint32_t psc = TIMx->PSC;

	
// 2. Check System CLK: PLL or HSI
	if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL)  		fsys = 84;  // for msec 84MHz/1000000 [usec]
	else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI) fsys = 16;

	
// 3. Configure prescaler PSC
	float fclk = fsys/(psc+1);					// fclk=fsys/(psc+1);
	uint32_t value = pulse_width_us *fclk -1;					// pulse_width_us *fclk - 1;
	
	switch(chN){
		case 1: TIMx->CCR1 = value; break;
		case 2: TIMx->CCR2 = value; break;
		case 3: TIMx->CCR3 = value; break;
		case 4: TIMx->CCR4 = value; break;
		// REPEAT for CHn=2,  3, 4

		default: break;
	}
}

// Dutry Ratio from 0 to 1 
void PWM_duty(PinName_t pinName, float duty){ 
// 0. Match TIMx from  Port and Pin 	
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(pinName, &port, &pin);	
	TIM_TypeDef *TIMx;
	int chN;		
	PWM_pinmap(pinName, &TIMx, &chN);

	unsigned int aRR=TIMx->ARR;
	

   
	
// 1. Configure prescaler PSC
	//float value = (aRR+1)*duty + 1; // (ARR+1)*dutyRatio + 1  
	//float value = (aRR+1)*duty;    								// (ARR+1)*dutyRatio
  unsigned int value = (aRR+1)*duty;    								// (ARR+1)*dutyRatio	
   	if(duty <= 0.f)
		value = 0;
	else if(duty >= 1.f)
		value = TIMx->ARR + 1;
  	if(chN == 1)      { TIMx->CCR1 = value; }          //set channel   
		else if(chN == 2)      { TIMx->CCR2 = value; }  //set channe2 
		else if(chN == 3)      { TIMx->CCR3 = value; }  //set channe3
		else if(chN == 4)      { TIMx->CCR4 = value; }  //set channe4 
	// REPEAT for CHn=2,  3, 4


}

// DO NOT MODIFY HERE
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN)
{
	GPIO_TypeDef *port;
	unsigned int pin;		
	ecPinmap(pinName, &port, &pin);	
		     
   
  if(port == GPIOA) {
      switch(pin){
         case 0 : *TIMx = TIM2; *chN	= 1; break;
         case 1 : *TIMx = TIM2; *chN = 2; break;
         case 5 : *TIMx = TIM2; *chN = 1; break;
         case 6 : *TIMx = TIM3; *chN = 1; break;
         //case 7: TIMx = TIM1; *chN = 1N; break;
         case 8 : *TIMx = TIM1; *chN = 1; break;
         case 9 : *TIMx = TIM1; *chN = 2; break;
         case 10: *TIMx = TIM1; *chN = 3; break;
         case 15: *TIMx = TIM2; *chN = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: TIMx = TIM1; *chN = 2N; break;
         //case 1: TIMx = TIM1; *chN = 3N; break;
         case 3 : *TIMx = TIM2; *chN = 2; break;
         case 4 : *TIMx = TIM3; *chN = 1; break;
         case 5 : *TIMx = TIM3; *chN = 2; break;
         case 6 : *TIMx = TIM4; *chN = 1; break;
         case 7 : *TIMx = TIM4; *chN = 2; break;
         case 8 : *TIMx = TIM4; *chN = 3; break;
         case 9 : *TIMx = TIM4; *chN = 4; break;
         case 10: *TIMx = TIM2; *chN = 3; break;     
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : *TIMx = TIM3; *chN = 1; break;
         case 7 : *TIMx = TIM3; *chN = 2; break;
         case 8 : *TIMx = TIM3; *chN = 3; break;
         case 9 : *TIMx = TIM3; *chN = 4; break;
         default: break;
      }
   }
	 // TIM5 needs to be added, if used.
}
