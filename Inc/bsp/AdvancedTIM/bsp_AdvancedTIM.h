#ifndef __ADVANCED_TIM_H__
#define __ADVANCED_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/

#define ADVANCED_TIMx                        TIM1
#define ADVANCED_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM1_CLK_ENABLE()
#define ADVANCED_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM1_CLK_DISABLE()
#define ADVANCED_TIM_GPIO_RCC_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE();
#define ADVANCED_TIM_CH1_PORT                GPIOA
#define ADVANCED_TIM_CH1_PIN                 GPIO_PIN_8



// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��72MHz/��ADVANCED_TIMx_PRESCALER+1��
#define ADVANCED_TIM_PRESCALER          71// 8MHZ //11 // ʱ��Ƶ�ʣ�6MHZ  //17 //4M  //35  // 2M   // 71  // ʵ��ʱ��Ƶ��Ϊ��1MHz

// ���嶨ʱ�����ڣ�����ʱ����ʼ������ADVANCED_TIMx_PERIODֵ�����ظ������Ĵ���Ϊ0ʱ���¶�ʱ�������ɶ�Ӧ�¼����ж�
#define ADVANCED_TIM_PERIOD               (1000-1)  // ��ʱ�������ж�Ƶ��Ϊ��1MHz/1000=1KHz����1ms��ʱ����
// ����߼���ʱ���ظ������Ĵ���ֵ��
#define ADVANCED_TIM_REPETITIONCOUNTER    0

// ���ն�ʱ��Ƶ�ʼ���Ϊ�� 72MHz/��ADVANCED_TIMx_PRESCALER+1/��ADVANCED_TIM_REPETITIONCOUNTER+1��/ADVANCED_TIMx_PERIOD
// ������Ҫ����20ms���ڶ�ʱ����������Ϊ�� 72MHz/��359+1��/��9+1��/400=50Hz����20ms����
// �������� ADVANCED_TIMx_PRESCALER=359��ADVANCED_TIM_REPETITIONCOUNTER=9��ADVANCED_TIMx_PERIOD=400��

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_advanced;

/* �������� ------------------------------------------------------------------*/

void ADVANCED_TIMx_Init(void);

#endif	/* __ADVANCED_TIM_H__ */

