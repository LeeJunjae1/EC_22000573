/**
******************************************************************************
* @author	Junjae Lee
* @Mod		09-10-2024
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Change LED by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC2_student.h"
#include "ecGPIO2_student.h"


#define LED_pin1 PA_6
#define LED_pin2 PA_7
#define LED_pin3 PB_6

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------

  int last_Button_state = 1; // initial button state
  int current_Button_state;
  int delay = 50; // delay in milliseconds
	int LED_state = 0;
	//state 0=led0 on
	//state 1=led1 on
	//state 2=led2 on
	//state 3=led3 on
	
	while(1){
		// Read the current button state
    current_Button_state = GPIO_read(button_pin);
		
		        // Check button (not pressed -> press)
        if (last_Button_state == 1 && current_Button_state == 0) {
					// Wait for debounce delay
					delay_time(delay);
					// Read button state again (button is still pressed)
          current_Button_state = GPIO_read(button_pin);
          if (current_Button_state == 0) {
						LED_state=LED_state%4;
						if(LED_state==0){
							GPIO_write(LED_pin1,LOW);//clear PA_6
							GPIO_write(LED_pin2,LOW);//clear PA_7
							GPIO_write(LED_pin,HIGH);//PA_5, PA_6, PA_7 ia all GPIOA, so only use one GPOI_write
							GPIO_write(LED_pin3,LOW);//PB_6 is use GPIOB
							LED_state++;//go state 1
						}
						else if(LED_state==1){
							GPIO_write(LED_pin,LOW);//clear PA_5
							GPIO_write(LED_pin2,LOW);//clear PA_7
							GPIO_write(LED_pin1,HIGH);
							GPIO_write(LED_pin3,LOW);
							LED_state++;//go state 2
						}
						else if(LED_state==2){
							GPIO_write(LED_pin,LOW);//clear PA_5
							GPIO_write(LED_pin1,LOW);//clear PA_6
							GPIO_write(LED_pin2,HIGH);
							GPIO_write(LED_pin3,LOW);
							LED_state++;//go state 3
						}
						else if(LED_state==3){
							GPIO_write(LED_pin,LOW);//clear PA_5
							GPIO_write(LED_pin1,LOW);//clear PA_6
							GPIO_write(LED_pin2,LOW);//clear PA_7
							GPIO_write(LED_pin3,HIGH);//set PB_6
							LED_state++;//go state 0
						}
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



// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	
  GPIO_init(button_pin, INPUT); // Calls RCC_GPIOC_enable()
  GPIO_pupd(button_pin, EC_UP); // pull-up
	
	//for led0
	GPIO_init(LED_pin, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(LED_pin, EC_UP);//PULL UP
	GPIO_otype(LED_pin, 0);//push-pull
	GPIO_ospeed(LED_pin, Medium_speed);//medium speed
	
	//for led1
	GPIO_init(LED_pin1, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(LED_pin1, EC_UP);//PULL UP
	GPIO_otype(LED_pin1, 0);//push-pull
	GPIO_ospeed(LED_pin1, Medium_speed);//medium speed
	
	//for led2
	GPIO_init(LED_pin2, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(LED_pin2, EC_UP);//PULL UP
	GPIO_otype(LED_pin2, 0);//push-pull
	GPIO_ospeed(LED_pin2, Medium_speed);//medium speed
	
	//for led3
	GPIO_init(LED_pin3, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(LED_pin3, EC_UP);//PULL UP
	GPIO_otype(LED_pin3, 0);//push-pull
	GPIO_ospeed(LED_pin3, Medium_speed);//medium speed
	
	//initialize
	GPIO_write(LED_pin, LOW);
	GPIO_write(LED_pin1, LOW);
	GPIO_write(LED_pin2, LOW);
	GPIO_write(LED_pin3, LOW);
}
