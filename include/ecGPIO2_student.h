/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 09-10-2024
Modified         : 09-21-2024
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#ifndef __ECGPIO2_H
#define __ECGPIO2_H

#include "stm32f411xe.h"
#include "ecRCC2_student.h"
#include "ecPinNames.h"

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define LED_pin PA_5    //Find LED Port&Pin and Fill the blank	
#define button_pin PC_13//Find BTN Port&Pin and Fill the blank

#define LED_PIN PA_5    //Find LED Port&Pin and Fill the blank	
#define BUTTON_PIN PC_13//Find BTN Port&Pin and Fill the blank

#define EC_PU  0X00 //non-pullup, non-pulldown
#define EC_UP  0X01 //pullup
#define EC_DOWN 0X02 //pulldown
#define EC_RE  0X03 //reversed

#define Low_speed  0x00
#define Medium_speed 0x01
#define Fast_speed   0x02
#define High_speed 0x03


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(PinName_t pinName, uint32_t mode);     
void GPIO_write(PinName_t pinName, int Output);
int  GPIO_read(PinName_t pinName);
void GPIO_mode(PinName_t pinName, uint32_t mode);
void GPIO_ospeed(PinName_t pinName, int speed);
void GPIO_otype(PinName_t pinName, int type);
void GPIO_pupd(PinName_t pinName, int pupd);

void delay_time(int ms);//delay function

void sevensegment_display_init(PinName_t pinNameA, PinName_t pinNameB, PinName_t pinNameC, PinName_t pinNameD); 
void sevensegment_display(uint8_t  num);
void sevensegment_decoder_init(void); 
void sevensegment_decoder(uint8_t  num);

//void GPIO_Decoder_clear(PinName_t pinName);// memorl clear
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __ECGPIO2_H
