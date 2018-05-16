#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdio.h>


//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	0		//0,��ʹ��;1,ʹ��.

uint8_t CAN1_Mode_Init(uint32_t tsjw,uint32_t tbs2,uint32_t tbs1,uint16_t brp,uint32_t mode);//CAN��ʼ��


uint8_t CAN1_Send_Msg(uint8_t* msg,uint8_t len,uint32_t exid);						//��������
uint8_t CAN1_Receive_Msg(uint8_t *buf,uint32_t *id);						//��������

#endif /* __BSP_CAN_H__ */



