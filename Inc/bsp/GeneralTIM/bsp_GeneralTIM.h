#ifndef __GENERAL_TIM_H__
#define __GENERAL_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"



/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
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

// 定义定时器预分频，定时器实际时钟频率为：72MHz/（GENERAL_TIMx_PRESCALER+1）
#define GENERAL_TIM_PRESCALER            8//17//4M//35  // 实际时钟频率为：2MHz

// 定义定时器周期，当定时器开始计数到GENERAL_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define GENERAL_TIM_PERIOD               100//1000  // 8/100=0.08Mhz// 定时器产生中断频率为?//?MHz/1000=2KHz，即0.5ms定时周期

//#define GENERAL_TIM_CH1_PULSE            60 // 定时器通道1占空比为：GENERAL_TIM_CH1_PULSE/GENERAL_TIM_PERIOD*100%=900/1000*100%=90%
#define GENERAL_TIM_CH2_PULSE            600   // 定时器通道2占空比为：GENERAL_TIM_CH2_PULSE/GENERAL_TIM_PERIOD*100%=600/1000*100%=60%
#define GENERAL_TIM_CH3_PULSE            300   // 定时器通道3占空比为：GENERAL_TIM_CH3_PULSE/GENERAL_TIM_PERIOD*100%=300/1000*100%=30%
//#define GENERAL_TIM_CH4_PULSE            600   // 定时器通道4占空比为：GENERAL_TIM_CH4_PULSE/GENERAL_TIM_PERIOD*100%=100/1000*100%=10%

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx;


/* 函数声明 ------------------------------------------------------------------*/
void GENERAL_TIMx_Init(void);

#endif	/* __GENERAL_TIM_H__ */
/******************* *****END OF FILE****/
