/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 09-25-2024
Modified         : 09-25-2024
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"

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

//for use 7-segment decoder
PinName_t PinNameA;
PinName_t PinNameB;
PinName_t PinNameC;
PinName_t PinNameD;



void sevensegment_display_init(PinName_t pinNameA, PinName_t pinNameB, PinName_t pinNameC, PinName_t pinNameD){
	
	//pinname array
	char PinName[4]={pinNameA, pinNameB, pinNameC, pinNameD};
	PinNameD=pinNameA;//decoder D pin
	PinNameC=pinNameB;//decoder C pin
	PinNameB=pinNameC;//decoder B pin
	PinNameA=pinNameD;//decoder A pin
	for(int i=0;i<4;i++){
		GPIO_init(PinName[i], OUTPUT); //Calls RCC_GPIO_enable(), pinName[i]'s mode: OUTPUT
	  GPIO_pupd(PinName[i], EC_PU);//No pull-up-pull-down
	  GPIO_otype(PinName[i], 0);//push-pull
	  GPIO_ospeed(PinName[i], Medium_speed);//medium speed
	}
	
}

/*
void sevensegment_display(uint8_t  num){
	int value[10][4]={
		//D,C,B,A
		{0,0,0,0},
		{0,0,0,1},
		{0,0,1,0},
		{0,0,1,1},
		{0,1,0,0},
		{0,1,0,1},
		{0,1,1,0},
		{0,1,1,1},
		{1,0,0,0},
		{1,0,0,1}
	};
	//{PA_7,PB_6,PC_7,PA_9};
	char pin_num[4]={PinNameD,PinNameC,PinNameB,PinNameA};
	for(int i=0;i<4;i++){
		GPIO_write(pin_num[i],value[num][i]);//PinNameD=PA_7=D, PinNameC=PB_6=C, PinNameB=PC_7=B, PinNameA=PA_9=A
	}
}*/

//minimize memory
void sevensegment_display(uint8_t num) {
    // D, C, B, A
    uint8_t value[10] = {
        0b0000, // 0
        0b0001, // 1
        0b0010, // 2
        0b0011, // 3
        0b0100, // 4
        0b0101, // 5
        0b0110, // 6
        0b0111, // 7
        0b1000, // 8
        0b1001  // 9
    };
		//{PA_7,PB_6,PC_7,PA_9};
    char pin_num[4]={PinNameD,PinNameC,PinNameB,PinNameA};
    
	//write value
    for (int i=0; i<4; i++) {
			//use bitwise
        GPIO_write(pin_num[3-i], (value[num]>>i)&0x01);//Create applicable pin content
    }
}




//decoder init
void sevensegment_decoder_init(void){
	//pinname array, using pins
	char decoder_ouput[8]={PA_5,PA_6,PA_7,PB_6,PC_7,PA_9,PA_8,PB_10};
	for(int i=0;i<8;i++){
		GPIO_init(decoder_ouput[i], OUTPUT); //Calls RCC_GPIOA_enable()
		GPIO_pupd(decoder_ouput[i], EC_PU);//No pull-up-pull-down
		GPIO_otype(decoder_ouput[i], 0);//push-pull
		GPIO_ospeed(decoder_ouput[i], Medium_speed);//medium speed
		
		//initialize to display 0
		if(i==6||i==7){
			GPIO_write(decoder_ouput[i],HIGH);//write 'HIGH'
		}
		else{
			GPIO_write(decoder_ouput[i],LOW);//write 'LOW'
		}
	}
}

/*
void GPIO_Decoder_clear(PinName_t pinName){
	GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
   Port->ODR &= 0;//clear
	 Port->ODR &= ~0;//set all 1
 
	
}*/

/*
void sevensegment_decoder(uint8_t  num){
	int value[10][8]={
		//a,b,c,d,e,f,g,h
		{0,0,0,0,0,0,1,1},//0
		{1,0,0,1,1,1,1,1},//1
		{0,0,1,0,0,1,0,1},//2
		{0,0,0,0,1,1,0,1},//3
		{1,0,0,1,1,0,0,1},//4
		{0,1,0,0,1,0,0,1},//5
		{0,1,0,0,0,0,0,1},//6
		{0,0,0,1,1,1,1,1},//7
		{0,0,0,0,0,0,0,1},//8
		{0,0,0,0,1,0,0,1}//9
	};
	char decoder_ouput[8]={PA_5,PA_6,PA_7,PB_6,PC_7,PA_9,PA_8,PB_10};
	for(int i=0;i<8;i++){
		GPIO_write(decoder_ouput[i],value[num][i]);
	}
}*/

//minimize memory
void sevensegment_decoder(uint8_t num) {
    // a, b, c, d, e, f, g, h
    uint8_t value[10] = {
        0b00000011, // 0
        0b10011111, // 1
        0b00100101, // 2
        0b00001101, // 3
        0b10011001, // 4
        0b01001001, // 5
        0b01000001, // 6
        0b00011111, // 7
        0b00000001, // 8
        0b00001001  // 9
    };
    //pinname array, using pins
    char decoder_output[8]={PA_5, PA_6, PA_7, PB_6, PC_7, PA_9, PA_8, PB_10};
    
    // Set output states based on the number
    for (int i=0; i<8; i++) {
			//use bitwise
        GPIO_write(decoder_output[7-i], (value[num]>>i)&0x01);//Create applicable pin content
    }
}
