/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-08-23 by YKKIM  	
* @brief   Embedded Controller:  Tutorial Digital In/Out 7-segment Display
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC2_student.h"
#include "ecGPIO2_student.h"

void setup(void);
	
int main(void) {	
	// Initialiization --------------------------------------------------------
	setup();
	uint8_t num=0;
  int last_Button_state = 1; // initial button state
  int current_Button_state;
  int delay = 50; // delay in milliseconds
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		current_Button_state = GPIO_read(button_pin);
		
		        // Check button (not pressed -> press)
        if (last_Button_state == 1 && current_Button_state == 0) {
					// Wait for debounce delay
					delay_time(delay);
					// Read button state again (button is still pressed)
          current_Button_state = GPIO_read(button_pin);
          if (current_Button_state == 0) {
						num=(num+1)%10;
						sevensegment_decoder(num);
					
						// Wait for button release
                while (GPIO_read(button_pin) == 0) {
                    // Wait here until button is released
									// If it is true->running
             }
						
					}	
				}
		    // Update the last button state
        last_Button_state = current_Button_state;
				
	}
}

void setup(void){
	RCC_HSI_init();
	
	GPIO_init(button_pin, INPUT); // Calls RCC_GPIOC_enable()
  GPIO_pupd(button_pin, EC_UP); // PULL UP
	
	sevensegment_decoder_init();
	
	/*
	GPIO_init(PA_5, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_5, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_5, 0);//push-pull
	GPIO_ospeed(PA_5, Medium_speed);//medium speed
	
	GPIO_init(PA_6, OUTPUT); //Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_6, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_6, 0);//push-pull
	GPIO_ospeed(PA_6, Medium_speed);//medium speed
	
	GPIO_init(PA_7, OUTPUT);//Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_7, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_7, 0);//push-pull
	GPIO_ospeed(PA_7, Medium_speed);//medium speed
	
	GPIO_init(PB_6, OUTPUT);//Calls RCC_GPIOB_enable()
	GPIO_pupd(PB_6, EC_PU);//No pull-up-pull-down
	GPIO_otype(PB_6, 0);//push-pull
	GPIO_ospeed(PB_6, Medium_speed);//medium speed
	
	GPIO_init(PC_7, OUTPUT);//Calls RCC_GPIOC_enable()
	GPIO_pupd(PC_7, EC_PU);//No pull-up-pull-down
	GPIO_otype(PC_7, 0);//push-pull
	GPIO_ospeed(PC_7, Medium_speed);//medium speed
	
	GPIO_init(PA_9, OUTPUT);//Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_9, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_9, 0);//push-pull
	GPIO_ospeed(PA_9, Medium_speed);//medium speed
	
	GPIO_init(PA_8, OUTPUT);//Calls RCC_GPIOA_enable()
	GPIO_pupd(PA_8, EC_PU);//No pull-up-pull-down
	GPIO_otype(PA_8, 0);//push-pull
	GPIO_ospeed(PA_8, Medium_speed);//medium speed
	
	GPIO_init(PB_10, OUTPUT);//Calls RCC_GPIOB_enable()
	GPIO_pupd(PB_10, EC_PU);//No pull-up-pull-down
	GPIO_otype(PB_10, 0);//push-pull
	GPIO_ospeed(PB_10, Medium_speed);//medium speed*/
}