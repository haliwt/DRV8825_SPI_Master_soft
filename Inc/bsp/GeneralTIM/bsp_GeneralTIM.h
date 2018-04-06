#ifndef __GENERAL_TIM_H__
#define __GENERAL_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"



/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
//#define GENERAL_TIMx                        TIM2
//#define GENERAL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM2_CLK_ENABLE()
//#define GENERAL_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM2_CLK_DISABLE()
//#define GENERAL_TIM_GPIO_RCC_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
//#define GENERAL_TIM_CH1_PORT                GPIOA
//#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_0
//#define GENERAL_TIM_CH2_PORT                GPIOA
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_1
//#define GENERAL_TIM_CH3_PORT                GPIOA
//#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_2
//#define GENERAL_TIM_CH4_PORT                GPIOA
//#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_3

//#define GENERAL_TIMx                        TIM3
//#define GENERAL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM3_CLK_ENABLE()
//#define GENERAL_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM3_CLK_DISABLE()
//#define GENERAL_TIM_GPIO_RCC_CLK_ENABLE()   {__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();}
//#define GENERAL_TIM_CH1_PORT                GPIOA
//#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_6
//#define GENERAL_TIM_CH2_PORT                GPIOA
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_7
//#define GENERAL_TIM_CH3_PORT                GPIOB
//#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_0
//#define GENERAL_TIM_CH4_PORT                GPIOB
//#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_1

#define GENERAL_TIMx                        TIM4
#define GENERAL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM4_CLK_ENABLE()
#define GENERAL_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM4_CLK_DISABLE()
#define GENERAL_TIM_GPIO_RCC_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define GENERAL_TIM_CH1_PORT                GPIOB
//#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_6
//#define GENERAL_TIM_CH2_PORT                GPIOB
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_7
//#define GENERAL_TIM_CH3_PORT                GPIOB
//#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_8
//#define GENERAL_TIM_CH4_PORT                GPIOB
#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_9

//#define GENERAL_TIMx                        TIM5
//#define GENERAL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM5_CLK_ENABLE()
//#define GENERAL_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM5_CLK_DISABLE()
//#define GENERAL_TIM_GPIO_RCC_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
//#define GENERAL_TIM_CH1_PORT                GPIOA
//#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_0
//#define GENERAL_TIM_CH2_PORT                GPIOA
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_1
//#define GENERAL_TIM_CH3_PORT                GPIOA
//#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_2
//#define GENERAL_TIM_CH4_PORT                GPIOA
//#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_3

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��72MHz/��GENERAL_TIMx_PRESCALER+1��
#define GENERAL_TIM_PRESCALER            8//17//4M//35  // ʵ��ʱ��Ƶ��Ϊ��2MHz

// ���嶨ʱ�����ڣ�����ʱ����ʼ������GENERAL_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define GENERAL_TIM_PERIOD               100//1000  // 8/100=0.08Mhz// ��ʱ�������ж�Ƶ��Ϊ?//?MHz/1000=2KHz����0.5ms��ʱ����

//#define GENERAL_TIM_CH1_PULSE            60 // ��ʱ��ͨ��1ռ�ձ�Ϊ��GENERAL_TIM_CH1_PULSE/GENERAL_TIM_PERIOD*100%=900/1000*100%=90%
#define GENERAL_TIM_CH2_PULSE            600   // ��ʱ��ͨ��2ռ�ձ�Ϊ��GENERAL_TIM_CH2_PULSE/GENERAL_TIM_PERIOD*100%=600/1000*100%=60%
#define GENERAL_TIM_CH3_PULSE            300   // ��ʱ��ͨ��3ռ�ձ�Ϊ��GENERAL_TIM_CH3_PULSE/GENERAL_TIM_PERIOD*100%=300/1000*100%=30%
//#define GENERAL_TIM_CH4_PULSE            600   // ��ʱ��ͨ��4ռ�ձ�Ϊ��GENERAL_TIM_CH4_PULSE/GENERAL_TIM_PERIOD*100%=100/1000*100%=10%

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx;


/* �������� ------------------------------------------------------------------*/
void GENERAL_TIMx_Init(void);

#endif	/* __GENERAL_TIM_H__ */
/******************* *****END OF FILE****/
