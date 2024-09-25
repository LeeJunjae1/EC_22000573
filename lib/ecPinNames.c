/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 09-10-2024
Modified         : 09-10-2024
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#include "ecPinNames.h"

void ecPinmap(PinName_t pinName, GPIO_TypeDef **GPIOx, unsigned int *pin)
{
	
	unsigned int pinNum= pinName & (0x000F);
	*pin=pinNum;

	unsigned int portNum=(pinName>>4);
	
	
	if (portNum==0)
		*GPIOx=GPIOA;
	else if (portNum==1)
		*GPIOx=GPIOB;
	else if (portNum==2)
		*GPIOx=GPIOC;
	else if (portNum==3)
		*GPIOx=GPIOD;
	else if (portNum==4)
		*GPIOx=GPIOE;
	else if (portNum==7)
		*GPIOx=GPIOH;
	else 
		*GPIOx=GPIOA;
}