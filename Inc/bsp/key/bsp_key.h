#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 类型定义 --------------------------------------------------------------*/
typedef enum
{
  KEY_UP   = 1,
  KEY_DOWN = 0,
}KEYState_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/
#define KEY1_RCC_CLK_ENABLE           __HAL_RCC_GPIOB_CLK_ENABLE
#define KEY1_GPIO_PIN                 GPIO_PIN_1     //WT.edit 2017.11.01
#define KEY1_GPIO                     GPIOB
#define KEY1_DOWN_LEVEL               0  /* 根据原理图设计，KEY1按下时引脚为高电平，所以这里设置为1 */
#define KEY1_EXTI_IRQn                EXTI1_IRQn
#define KEY1_EXTI_IRQHandler          EXTI1_IRQHandler


#define KEY2_RCC_CLK_ENABLE           __HAL_RCC_GPIOB_CLK_ENABLE
#define KEY2_GPIO_PIN                 GPIO_PIN_10  
#define KEY2_GPIO                     GPIOB
#define KEY2_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */
#define KEY2_EXTI_IRQHandler          EXTI15_10_IRQHandler
#define KEY2_EXTI_IRQn                EXTI15_10_IRQn


#define KEY3_RCC_CLK_ENABLE           __HAL_RCC_GPIOB_CLK_ENABLE
#define KEY3_GPIO_PIN                 GPIO_PIN_0
#define KEY3_GPIO                     GPIOB
#define KEY3_DOWN_LEVEL               0  /* 根据原理图设计，KEY1按下时引脚为高电平，所以这里设置为1 */
#define KEY3_EXTI_IRQn                EXTI0_IRQn
#define KEY3_EXTI_IRQHandler          EXTI0_IRQHandler

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void KEY_GPIO_Init(void);
KEYState_TypeDef KEY1_StateRead(void);
KEYState_TypeDef KEY2_StateRead(void);
KEYState_TypeDef KEY3_StateRead(void);


#endif  // __BSP_KEY_H__


