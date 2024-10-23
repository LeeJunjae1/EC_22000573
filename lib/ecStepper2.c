/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 10-23-2024
Modified         : 10-23-2024
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "ecStepper2.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;
    

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
   uint8_t out;
     uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {     // 1010 , 0110 , 0101 , 1001
    {0b1100,{S1,S3}},      // ABA'B'
   {0b0110,{S2,S0}},
   {0b0011,{S3,S1}},
   {0b1001,{S0,S2}}
   
   /*
   {0b1100,{S3,S1}},      // ABA'B'
   {0b0110,{S0,S2}},
   {0b0011,{S1,S3}},
   {0b1001,{S2,S4}}
   */
};

//HALF stepping sequence
typedef struct {
   uint8_t out;
     uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = {   // 1000 , 1010 , 0010 , 0110 , 0100 , 0101, 0001, 1001
    //ABA'B'
   {0b1000,{S1,S7}},   
   {0b1100,{S2,S0}},   
   {0b0100,{S3,S1}},   
   {0b0110,{S4,S2}},   
   {0b0010,{S5,S3}},   
   {0b0011,{S6,S4}},   
   {0b0001,{S7,S5}},   
   {0b1001,{S0,S6}}
   
   /*
   {0b1000,{S7,S1}},   
   {0b1100,{S0,S2}},   
   {0b0100,{S1,S3}},   
   {0b0110,{S2,S4}},   
   {0b0010,{S3,S5}},   
   {0b0011,{S4,S6}},   
   {0b0001,{S5,S7}},   
   {0b1001,{S6,S0}}*/
};



void Stepper_init(PinName_t pinName1, PinName_t pinName2, PinName_t pinName3, PinName_t pinName4){
    
   //  GPIO Digital Out Initiation
   myStepper.pin1 = pinName1;
   myStepper.pin2 = pinName2;
   myStepper.pin3 = pinName3;
   myStepper.pin4 = pinName4;

   
   //  GPIO Digital Out Initiation
   // No pull-up Pull-down , Push-Pull, Fast   
   char PinName[4]={pinName1, pinName2, pinName3, pinName4};
   //char PinName[4]={myStepper.pin1, myStepper.pin2, myStepper.pin3, myStepper.pin4};
   for(int i=0;i<4;i++){
      GPIO_init(PinName[i], OUTPUT); //Calls RCC_GPIO_enable(), pinName[i]'s mode: OUTPUT
     GPIO_pupd(PinName[i], EC_PU);//No pull-up-pull-down
     GPIO_otype(PinName[i], 0);//push-pull
     GPIO_ospeed(PinName[i], Fast_speed);//medium speed
   }
   
   
   // Pin1 ~ Port4
}


void Stepper_pinOut (uint32_t state, uint32_t mode){   
      if (mode == FULL){         // FULL mode
         /*
               GPIO_write(myStepper.pin1, (FSM_full[state].out & 0b1000) >> 3);
      GPIO_write(myStepper.pin3, (FSM_full[state].out & 0b0100) >> 2);
      GPIO_write(myStepper.pin3, (FSM_full[state].out & 0b0010) >> 1);
      GPIO_write(myStepper.pin4, (FSM_full[state].out & 0b0001) >> 0);
         */
      GPIO_write(myStepper.pin1, (FSM_full[state].out >>3) &1);//use bitwise
      GPIO_write(myStepper.pin2, (FSM_full[state].out >>2) &1);
      GPIO_write(myStepper.pin3, (FSM_full[state].out >>1) &1);
      GPIO_write(myStepper.pin4, (FSM_full[state].out >>0) &1);
        // Repeat for pin2~port4 
   }    
    else if (mode == HALF){    // HALF mode
      GPIO_write(myStepper.pin1, (FSM_half[state].out >>3) &1);
      GPIO_write(myStepper.pin2, (FSM_half[state].out >>2) &1);
      GPIO_write(myStepper.pin3, (FSM_half[state].out >>1) &1);
      GPIO_write(myStepper.pin4, (FSM_half[state].out >>0) &1);

   }
}


void Stepper_setSpeed (long whatSpeed){      // rpm [rev/min]
      //1 rotation->
      //step_delay =    whatSpeed/step_per_rev;//YOUR CODE   // Convert rpm to  [msec/step] delay
	
      step_delay=60*1000/(step_per_rev*whatSpeed);//use msec
			//step_delay=60*1000000/(step_per_rev)/(whatSpeed);//use usec, more accurate
      //step_delay=100;
}


void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode){
    uint32_t state = 0;
    myStepper._step_num = steps;

    for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
                            // delay (step_delay);              
       delay_ms(step_delay);
          if (mode == FULL) //full mode                                           
         state = FSM_full[state].next[direction];      // state = next state
      else if (mode == HALF) //half mode
         state = FSM_half[state].next[direction];       // state = next state      
      Stepper_pinOut(state, mode);
      }
}


void Stepper_stop (void){ 
       myStepper._step_num = 0;    
   // All pins(A,AN,B,BN) set as DigitalOut '0'
	
	//stop
   GPIO_write(myStepper.pin1, 0);
   GPIO_write(myStepper.pin2, 0);
   GPIO_write(myStepper.pin3, 0);
   GPIO_write(myStepper.pin4, 0);
	//delay_ms(100);

}

