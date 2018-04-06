#ifndef __BSP_LAMP
#define __BSP_LAMP

#include "stm32f1xx_hal.h"

void LAMP_Save_BrightValue(uint8_t bv);

uint8_t LAMP_Read_BrightValue(void);

uint8_t A2_LAMP_Read_BrightValue(void);

void PC_A2_Pulse_EEPROM_Value(void);

#endif
