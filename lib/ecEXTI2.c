#include "ecGPIO2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"


void EXTI_init(PinName_t pinName, int trig_type,int priority){

	GPIO_TypeDef * Port;
	unsigned int Pin;
	ecPinmap(pinName,&Port,&Pin);
	// SYSCFG peripheral clock enable	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		
	
	// Connect External Line to the GPIO
	int EXTICR_port;
	if			(Port == GPIOA) EXTICR_port = 0;
	else if	(Port == GPIOB) EXTICR_port = 1;
	else if	(Port == GPIOC) EXTICR_port = 2;
	else if	(Port == GPIOD) EXTICR_port = 3;
	else if	(Port == GPIOE) EXTICR_port = 4;
	else 										EXTICR_port = 7;
	
	unsigned int pin=Pin%4;// use pin number of EXTICR
	unsigned int Re_num=Pin/4;//EXTICR number
	SYSCFG->EXTICR[Re_num] &= ~(0xF<<(pin*4));			// clear 4 bits
	SYSCFG->EXTICR[Re_num] |= (EXTICR_port<<(pin*4));			// set 4 bits
	
	// Configure Trigger edge
	if (trig_type == FALL) EXTI->FTSR |= (1<<Pin);   // Falling trigger enable 
	else if	(trig_type == RISE) EXTI->RTSR |= (1<<Pin);   // Rising trigger enable 
	else if	(trig_type == BOTH) {			// Both falling/rising trigger enable
		EXTI->RTSR |= (1<<Pin); 
		EXTI->FTSR |=(1<<Pin);
	} 
	
	// Configure Interrupt Mask (Interrupt enabled)
	EXTI->IMR  |= (1<<Pin);     // not masked
	
	
	// NVIC(IRQ) Setting
	int EXTI_IRQn = 0;
	
	if (Pin < 5) 	EXTI_IRQn = Pin+6;//Poistion
	else if	(Pin < 10) 	EXTI_IRQn = EXTI9_5_IRQn;
	else 			EXTI_IRQn = EXTI15_10_IRQn;
								
	NVIC_SetPriority(EXTI_IRQn, priority);	// EXTI priority
	NVIC_EnableIRQ(EXTI_IRQn); 	// EXTI IRQ enable
}


void EXTI_enable(PinName_t pinName) {
	//Todo Port, Pin Configuration 
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName,&Port,&pin);
	EXTI->IMR |= (1<<pin);     // not masked (i.e., Interrupt enabled)
}
void EXTI_disable(PinName_t pinName) {
	//Todo Port, Pin Configuration 
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName,&Port,&pin);
	EXTI->IMR &=~(1<<pin);     // masked (i.e., Interrupt disabled)
}

uint32_t is_pending_EXTI(PinName_t pinName) {
	//Todo Port, Pin Configuration 
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName,&Port,&pin);
	uint32_t EXTI_PRx = (1<<pin);     	// check  EXTI pending 	
	return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);
}


void clear_pending_EXTI(PinName_t pinName) {
	//Todo Port, Pin Configuration 
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName,&Port,&pin);
	EXTI->PR|= (1<<pin);     // clear EXTI pending 
}
