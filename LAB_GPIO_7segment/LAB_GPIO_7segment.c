/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2024-09-20 by JunjaeLee  	
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
						
						//Problem 2
						//7-segment decoder code
						//sevensegment_decoder(num); 
						
						//Problem 1
						//7-segment display code
						sevensegment_display(num);
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
	
	//Problem 2
	//7-segment decoder code
	//sevensegment_decoder_init();
	
	//Problem 1
	//7-segment display code
	sevensegment_display_init(PA_7, PB_6, PC_7, PA_9); 
}