/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 09-10-2024
Modified         : 09-10-2024
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO2_student.h"
#include "ecRCC2_student.h"

void RCC_GPIO_enable(PinName_t pinName){
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName, &Port, &pin);
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOB)
			RCC_GPIOB_enable();
	if (Port == GPIOD)
			RCC_GPIOD_enable();
	if (Port == GPIOE)
			RCC_GPIOE_enable();
	if (Port == GPIOH)
			RCC_GPIOH_enable();
}



void delay_time(int ms) {
    volatile int count;
    for (count = 0; count < ms * 1000; count++) {
    }
}

void GPIO_init(PinName_t pinName, uint32_t mode){     
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName, &Port, &pin);
	//RCC_GPIO_enable(Port);
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	/*if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	
	//[TO-DO] YOUR CODE GOES HERE
	// Make it for GPIOB, GPIOD..GPIOH
	
	if (Port == GPIOB)
			RCC_GPIOB_enable();
	if (Port == GPIOD)
			RCC_GPIOD_enable();
	if (Port == GPIOE)
			RCC_GPIOE_enable();
	if (Port == GPIOH)
			RCC_GPIOH_enable();*/
	
	
	// You can also make a more general function of
	RCC_GPIO_enable(pinName); 

	GPIO_mode(pinName, mode);
}



// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(PinName_t pinName, uint32_t mode){
   GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(PinName_t pinName, int speed){
	 GPIO_TypeDef * Port;
	 unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
   Port->OSPEEDR &= ~(3UL<<(2*pin));  
	 Port->OSPEEDR |= speed<<(2*pin);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(PinName_t pinName, int type){
	 GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
   Port->OTYPER &= ~(1UL<<(pin));
	 Port->OTYPER |= type<<(pin);  
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(PinName_t pinName, int pupd){
	GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
   Port->PUPDR &= ~(3UL<<(2*pin));  
	 Port->PUPDR |= pupd<<(2*pin);
}

int GPIO_read(PinName_t pinName){
	 GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
	unsigned int out;
	out=Port->IDR>>pin&1;
	return out;//[TO-DO] YOUR CODE GOES HERE	
}

void GPIO_write(PinName_t pinName, int Output){
	GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
   //Port->ODR &= ~(1UL<<(pin));
//	 Port->ODR |= Output<<(pin); 
 if(Output==HIGH){  
	 Port->ODR |= Output<<(pin);
 }
 else{
	 Port->ODR &= ~(1<<(pin));
 }
	
}

void sevensegment_display_init(PinName_t pinNameA, PinName_t pinNameB, PinName_t pinNameC, PinName_t pinNameD){
	/*
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinNameA, &Port, &pin);
	RCC_GPIO_enable(pinNameA); //pinNameA enable
	
	ecPinmap(pinNameB, &Port, &pin);
	RCC_GPIO_enable(pinNameB); //pinNameB enable
	
	ecPinmap(pinNameC, &Port, &pin);
	RCC_GPIO_enable(pinNameC); //pinNameC enable
	
	ecPinmap(pinNameD, &Port, &pin);
	RCC_GPIO_enable(pinNameD); //pinNameD enable*/
	GPIO_init(pinNameA, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(pinNameA, EC_PU);//No pull-up-pull-down
	GPIO_otype(pinNameA, 0);//push-pull
	GPIO_ospeed(pinNameA, Medium_speed);//medium speed
	
	GPIO_init(pinNameB, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(pinNameB, EC_PU);//No pull-up-pull-down
	GPIO_otype(pinNameB, 0);//push-pull
	GPIO_ospeed(pinNameB, Medium_speed);//medium speed
	
	GPIO_init(pinNameC, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(pinNameC, EC_PU);//No pull-up-pull-down
	GPIO_otype(pinNameC, 0);//push-pull
	GPIO_ospeed(pinNameC, Medium_speed);//medium speed
	
	GPIO_init(pinNameD, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(pinNameD, EC_PU);//No pull-up-pull-down
	GPIO_otype(pinNameD, 0);//push-pull
	GPIO_ospeed(pinNameD, Medium_speed);//medium speed
	
}

void sevensegment_display(uint8_t  num){
	if(num==0){//0b0000
		GPIO_write(PA_7,LOW);//PA_7=D
		GPIO_write(PB_6,LOW);//PB_6=C
		GPIO_write(PC_7,LOW);//PC_7=B
		GPIO_write(PA_9,LOW);//PA_9=A
	}
	else if(num==1){//0b0001
		GPIO_write(PA_7,LOW);//PA_7=D
		GPIO_write(PB_6,LOW);//PB_6=C
		GPIO_write(PC_7,LOW);//PC_7=B
		GPIO_write(PA_9,HIGH);//PA_9=A
	}
	else if(num==2){//0b0010
		GPIO_write(PA_7,LOW);//PA_7=D
		GPIO_write(PB_6,LOW);//PB_6=C
		GPIO_write(PC_7,HIGH);//PC_7=B
		GPIO_write(PA_9,LOW);//PA_9=A
	}
	else if(num==3){//0b0011
		GPIO_write(PA_7,LOW);//PA_7=D
		GPIO_write(PB_6,LOW);//PB_6=C
		GPIO_write(PC_7,HIGH);//PC_7=B
		GPIO_write(PA_9,HIGH);//PA_9=A
	}
	else if(num==4){//0b0100
		GPIO_write(PA_7,LOW);//PA_7=D
		GPIO_write(PB_6,HIGH);//PB_6=C
		GPIO_write(PC_7,LOW);//PC_7=B
		GPIO_write(PA_9,LOW);//PA_9=A
	}
	else if(num==5){//0b0101
		GPIO_write(PA_7,LOW);//PA_7=D
		GPIO_write(PB_6,HIGH);//PB_6=C
		GPIO_write(PC_7,LOW);//PC_7=B
		GPIO_write(PA_9,HIGH);//PA_9=A
	}
	else if(num==6){//0b0110
		GPIO_write(PA_7,LOW);//PA_7=D
		GPIO_write(PB_6,HIGH);//PB_6=C
		GPIO_write(PC_7,HIGH);//PC_7=B
		GPIO_write(PA_9,LOW);//PA_9=A
	}
	else if(num==7){//0b0111
		GPIO_write(PA_7,LOW);//PA_7=D
		GPIO_write(PB_6,HIGH);//PB_6=C
		GPIO_write(PC_7,HIGH);//PC_7=B
		GPIO_write(PA_9,HIGH);//PA_9=A
	}
	else if(num==8){//0b1000
		GPIO_write(PA_7,HIGH);//PA_7=D
		GPIO_write(PB_6,LOW);//PB_6=C
		GPIO_write(PC_7,LOW);//PC_7=B
		GPIO_write(PA_9,LOW);//PA_9=A
		
	}
	else if(num==9){//0b1001
		GPIO_write(PA_7,HIGH);//PA_7=D
		GPIO_write(PB_6,LOW);//PB_6=C
		GPIO_write(PC_7,LOW);//PC_7=B
		GPIO_write(PA_9,HIGH);//PA_9=A
		
	}
	
}

//decoder init
void sevensegment_decoder_init(void){
	//a
	GPIO_init(PA_5, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_5, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_5, 0);//push-pull
	GPIO_ospeed(PA_5, Medium_speed);//medium speed
	
	//b
	GPIO_init(PA_6, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_6, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_6, 0);//push-pull
	GPIO_ospeed(PA_6, Medium_speed);//medium speed
	
	//c
	GPIO_init(PA_7, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_7, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_7, 0);//push-pull
	GPIO_ospeed(PA_7, Medium_speed);//medium speed
	
	//d
	GPIO_init(PB_6, OUTPUT); //Calls RCC_GPIOB_enable()
	GPIO_pupd(PB_6, EC_PU);//No pull-up-pull-down
	GPIO_otype(PB_6, 0);//push-pull
	GPIO_ospeed(PB_6, Medium_speed);//medium speed
	
	//e
	GPIO_init(PC_7, OUTPUT); //Calls RCC_GPIOC_enable()
	GPIO_pupd(PC_7, EC_PU);//No pull-up-pull-down
	GPIO_otype(PC_7, 0);//push-pull
	GPIO_ospeed(PC_7, Medium_speed);//medium speed
	
	//f
	GPIO_init(PA_9, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_9, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_9, 0);//push-pull
	GPIO_ospeed(PA_9, Medium_speed);//medium speed
	
	//g
	GPIO_init(PA_8, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_8, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_8, 0);//push-pull
	GPIO_ospeed(PA_8, Medium_speed);//medium speed
	
	//h
	GPIO_init(PB_10, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(PB_10, EC_PU);//No pull-up-pull-down
	GPIO_otype(PB_10, 0);//push-pull
	GPIO_ospeed(PB_10, Medium_speed);//medium speed
	
	GPIO_write(PA_5,LOW);//PA_5=a
	GPIO_write(PA_6,LOW);//PA_6=b
	GPIO_write(PA_7,LOW);//PA_7=c
	GPIO_write(PB_6,LOW);//PB_6=d
	GPIO_write(PC_7,LOW);//PC_7=e
	GPIO_write(PA_9,LOW);//PA_9=f
	GPIO_write(PA_8,HIGH);//PA_8=g
	GPIO_write(PB_10,HIGH);//PB_10=h, vcc
		/*
		GPIO_Decoder_clear(PA_5);//GPIOA clear
		GPIO_Decoder_clear(PB_6);//GPIOB clear
		GPIO_Decoder_clear(PC_7);//GPIOC clear*/
}

void GPIO_Decoder_clear(PinName_t pinName){
	GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
   Port->ODR &= 0;//clear
	 Port->ODR &= ~0;//set all 1
 
	
}


void sevensegment_decoder(uint8_t  num){
	
	if(num==0){//1111110
		GPIO_write(PA_5,LOW);//PA_5=a
		GPIO_write(PA_6,LOW);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,LOW);//PB_6=d
		GPIO_write(PC_7,LOW);//PC_7=e
		GPIO_write(PA_9,LOW);//PA_9=f
		GPIO_write(PA_8,HIGH);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h, vcc
	}
	else if(num==1){//0110000
		GPIO_write(PA_5,HIGH);//PA_5=a
		GPIO_write(PA_6,LOW);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,HIGH);//PB_6=d
		GPIO_write(PC_7,HIGH);//PC_7=e
		GPIO_write(PA_9,HIGH);//PA_9=f
		GPIO_write(PA_8,HIGH);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
	}
	else if(num==2){//1101101
		GPIO_write(PA_5,LOW);//PA_5=a
		GPIO_write(PA_6,LOW);//PA_6=b
		GPIO_write(PA_7,HIGH);//PA_7=c
		GPIO_write(PB_6,LOW);//PB_6=d
		GPIO_write(PC_7,LOW);//PC_7=e
		GPIO_write(PA_9,HIGH);//PA_9=f
		GPIO_write(PA_8,LOW);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
	}
	else if(num==3){//1111001
		GPIO_write(PA_5,LOW);//PA_5=a
		GPIO_write(PA_6,LOW);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,LOW);//PB_6=d
		GPIO_write(PC_7,HIGH);//PC_7=e
		GPIO_write(PA_9,HIGH);//PA_9=f
		GPIO_write(PA_8,LOW);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
	}
	else if(num==4){//0110011
		GPIO_write(PA_5,HIGH);//PA_5=a
		GPIO_write(PA_6,LOW);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,HIGH);//PB_6=d
		GPIO_write(PC_7,HIGH);//PC_7=e
		GPIO_write(PA_9,LOW);//PA_9=f
		GPIO_write(PA_8,LOW);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
	}
	else if(num==5){//1011011
		GPIO_write(PA_5,LOW);//PA_5=a
		GPIO_write(PA_6,HIGH);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,LOW);//PB_6=d
		GPIO_write(PC_7,HIGH);//PC_7=e
		GPIO_write(PA_9,LOW);//PA_9=f
		GPIO_write(PA_8,LOW);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
	}
	else if(num==6){//1011111
		GPIO_write(PA_5,LOW);//PA_5=a
		GPIO_write(PA_6,HIGH);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,LOW);//PB_6=d
		GPIO_write(PC_7,LOW);//PC_7=e
		GPIO_write(PA_9,LOW);//PA_9=f
		GPIO_write(PA_8,LOW);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
	}
	else if(num==7){//1110000
		GPIO_write(PA_5,LOW);//PA_5=a
		GPIO_write(PA_6,LOW);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,HIGH);//PB_6=d
		GPIO_write(PC_7,HIGH);//PC_7=e
		GPIO_write(PA_9,HIGH);//PA_9=f
		GPIO_write(PA_8,HIGH);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
	}
	else if(num==8){//1111111
		GPIO_write(PA_5,LOW);//PA_5=a
		GPIO_write(PA_6,LOW);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,LOW);//PB_6=d
		GPIO_write(PC_7,LOW);//PC_7=e
		GPIO_write(PA_9,LOW);//PA_9=f
		GPIO_write(PA_8,LOW);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
		
	}
	else if(num==9){//1111011
		GPIO_write(PA_5,LOW);//PA_5=a
		GPIO_write(PA_6,LOW);//PA_6=b
		GPIO_write(PA_7,LOW);//PA_7=c
		GPIO_write(PB_6,LOW);//PB_6=d
		GPIO_write(PC_7,HIGH);//PC_7=e
		GPIO_write(PA_9,LOW);//PA_9=f
		GPIO_write(PA_8,LOW);//PA_8=g
		GPIO_write(PB_10,HIGH);//PB_10=h
		
	}
}
