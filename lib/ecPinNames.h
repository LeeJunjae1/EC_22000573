/*----------------------------------------------------------------\
@ Embedded Controller by Junjae Lee - Handong Global University
Author           : SSS LAB
Created          : 09-10-2024
Modified         : 09-10-2024
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/


#ifndef EC_PINNAMES_H
#define EC_PINNAMES_H

#include "stm32f411xe.h"

#ifdef __cplusplus
extern "C" {
#endif


// Bitwise Macro Definition
#define BIT_SET(REG, BIT)      	((REG) |= 1<< (BIT))
#define BIT_CLEAR(REG, BIT)     ((REG) &= ~1<<(BIT))
#define BIT_READ(REG, BIT)      ((REG)>>BIT & (1))
#define BITS_SET(REG, BIT,NUM)     ((REG) |= NUM<< (BIT))
#define BITS_CLEAR(REG, BIT,NUM)   ((REG) &= ~(NUM<< (BIT))
//#define BITS_CLEAR(REG, BIT,NUM)   ((REG) &= ~((0x1<< NUM)-1)<<(BIT))


// Pinname Config
typedef enum {
    PortA = 0,
    PortB = 1,
    PortC = 2,
    PortD = 3,
    PortE = 4,
    PortF = 5,
    PortG = 6,
    PortH = 7,
    PortI = 8,
    PortJ = 9,
    PortK = 10
} PortName_t;



typedef enum {
    PA_0  = 0x00,
    PA_1  = 0x01,    
    PA_2  = 0x02,
    PA_3  = 0x03,
    PA_4  = 0x04,    
    PA_5  = 0x05,
    PA_6  = 0x06,
    PA_7  = 0x07,
    PA_8  = 0x08,
    PA_9  = 0x09,
    PA_10 = 0x0A,
    PA_11 = 0x0B,
    PA_12 = 0x0C,
    PA_13 = 0x0D,
    PA_14 = 0x0E,
    PA_15 = 0x0F,
    
    PB_0  = 0x10,
    PB_1  = 0x11,
    PB_2  = 0x12,
    PB_3  = 0x13,
    PB_4  = 0x14,
    PB_5  = 0x15,
    PB_6  = 0x16,
    PB_7  = 0x17,
    PB_8  = 0x18,
    PB_9  = 0x19,
    PB_10 = 0x1A,
    PB_12 = 0x1C,
    PB_13 = 0x1D,
    PB_14 = 0x1E,
    PB_15 = 0x1F,

    PC_0  = 0x20,
    PC_1  = 0x21,
    PC_2  = 0x22,
    PC_3  = 0x23,
    PC_4  = 0x24,
    PC_5  = 0x25,
    PC_6  = 0x26,
    PC_7  = 0x27,
    PC_8  = 0x28,
    PC_9  = 0x29,
    PC_10 = 0x2A,
    PC_11 = 0x2B,
    PC_12 = 0x2C,
    PC_13 = 0x2D,
    PC_14 = 0x2E,
    PC_15 = 0x2F,

    PD_0  = 0x30,
		PD_1  = 0x31,
		PD_2  = 0x32,
		PD_3  = 0x33,
		PD_4  = 0x34,
		PD_5  = 0x35,
		PD_6  = 0x36,
		PD_7  = 0x37,
		PD_8  = 0x38,
		PD_9  = 0x39,
		PD_10 = 0x3A,
		PD_11 = 0x3B,
		PD_12 = 0x3C,
		PD_13 = 0x3D,
    PD_14 = 0x3E,
    PD_15 = 0x3F,

    PE_0  = 0x40,
		PE_1  = 0x41,
		PE_2  = 0x42,
		PE_3  = 0x43,
		PE_4  = 0x44,
		PE_5  = 0x45,
		PE_6  = 0x46,
		PE_7  = 0x47,
		PE_8  = 0x48,
		PE_9  = 0x49,
		PE_10 = 0x4A,
		PE_11 = 0x4B,
		PE_12 = 0x4C,
		PE_13 = 0x4D,
    PE_14 = 0x4E,
    PE_15 = 0x4F,

    PH_0  = 0x70,
    PH_1  = 0x71,
		PH_2  = 0x72,
		PH_3  = 0x73,
		PH_4  = 0x74,
		PH_5  = 0x75,
		PH_6  = 0x76,
		PH_7  = 0x77,
		PH_8  = 0x78,
		PH_9  = 0x79,
		PH_10 = 0x7A,
		PH_11 = 0x7B,
		PH_12 = 0x7C,
		PH_13 = 0x7D,
    PH_14 = 0x7E,
    PH_15 = 0x7F,


    // Arduino connector namings
    A0          = PA_0,
    A1          = PA_1,
    A2          = PA_4,
    A3          = PB_0,
    A4          = PC_1,
    A5          = PC_0,
    D0          = PA_3,
    D1          = PA_2,
    D2          = PA_10,
    D3          = PB_3,
    D4          = PB_5,
    D5          = PB_4,
    D6          = PB_10,
    D7          = PA_8,
    D8          = PA_9,
    D9          = PC_7,
    D10         = PB_6,
    D11         = PA_7,
    D12         = PA_6,
    D13         = PA_5,
    D14         = PB_9,
    D15         = PB_8,


    // Not connected
    NC = (int)0xFFFFFFFF
} PinName_t;


void ecPinmap(PinName_t pinName, GPIO_TypeDef **GPIOx, unsigned int *pin);




#ifdef __cplusplus
}
#endif

#endif
