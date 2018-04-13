#ifndef _HEXTODEC_H
#define _HEXTODEC_H
#include "stm32f1xx_hal.h"

//const uint8_t hex[16]={0,1,2,3,4,5,6,7,8,9,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
//const uint8_t dec[16]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

uint32_t  Hex2oct_MSB(uint32_t  msb);
uint32_t  Hex2oct_LSB(uint32_t  lsb);
uint32_t  Hex2oct_MD1(uint32_t  md1);
uint32_t  Hex2oct_MD2(uint32_t  md2);



#endif

