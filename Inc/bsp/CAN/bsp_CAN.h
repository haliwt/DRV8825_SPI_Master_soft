#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdio.h>


//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	0		//0,不使能;1,使能.

uint8_t CAN1_Mode_Init(uint32_t tsjw,uint32_t tbs2,uint32_t tbs1,uint16_t brp,uint32_t mode);//CAN初始化


uint8_t CAN1_Send_Msg(uint8_t* msg,uint8_t len,uint32_t exid);						//发送数据
uint8_t CAN1_Receive_Msg(uint8_t *buf,uint32_t *id);						//接收数据

#endif /* __BSP_CAN_H__ */



