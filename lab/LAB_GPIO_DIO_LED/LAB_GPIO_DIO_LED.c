/**
******************************************************************************
* @author	Junjae Lee
* @Mod		09-10-2024
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED LD2 by Button B1 pressing
* 
******************************************************************************
*/



#include "stm32f4xx.h"
#include "ecRCC2.h"
#include "ecGPIO2.h"


// Function prototypes
void setup(void);


int main(void) { 
    // Initialization
    setup();
    
    // Variables for button state and LED management
    int LED_state = 0; // 0 = LED off, 1 = LED on
    int last_Button_state = 1; // initial button state
    int current_Button_state;
    int delay = 50; // delay in milliseconds

    while (1) {
        // Read the current button state
        current_Button_state = GPIO_read(button_pin);

        // Check button (not pressed -> press)
        if (last_Button_state == 1 && current_Button_state == 0) {
          // Wait for debounce delay
          delay_time(delay);
            
          // Read button state again (button is still pressed)
          current_Button_state = GPIO_read(button_pin);
          if (current_Button_state == 0) {
                // Toggle LED state
              LED_state = !LED_state; // Toggle LED state
							if(LED_state==HIGH){//led state=0(led off state)->1
								GPIO_write(LED_pin,HIGH);
							}
							else{//led state=1(led on state)->0 
                GPIO_write(LED_pin, LOW);
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

// Initialization function
void setup(void) {
    RCC_HSI_init();
    
    // Initialize button pin
    GPIO_init(button_pin, INPUT); // Calls RCC_GPIOC_enable()
    GPIO_pupd(button_pin, EC_UP); // pull-up

    // Initialize LED pin
    GPIO_init(LED_pin, OUTPUT); // Calls RCC_GPIOA_enable()
    GPIO_pupd(LED_pin, EC_UP); // Pull-up
    GPIO_otype(LED_pin, 0); // Push-pull
    GPIO_ospeed(LED_pin, Medium_speed); // Medium speed

    // initial 
    GPIO_write(LED_pin, LOW);
}