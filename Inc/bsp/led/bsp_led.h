#ifndef __BSP_LED_H__
#define __BSP_LED_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum
{
  LED_OFF = 0,
  LED_ON  = 1,
  LED_TOGGLE = 2,
}LEDState_TypeDef;
#define IS_LED_STATE(STATE)           (((STATE) == LED_OFF) || ((STATE) == LED_ON) || ((STATE) == LED_TOGGLE))

/* �궨�� --------------------------------------------------------------------*/
#define LED1                          (uint8_t)0x01
#define LED2                          (uint8_t)0x02
#define LED3                          (uint8_t)0x04
#define IS_LED_TYPEDEF(LED)           (((LED) == LED1) || ((LED) == LED2) || ((LED) == LED3))

/*
 * ���º궨�����ݸ�������Ӳ��ϢϢ��أ���Ҫ�鿴�������·ԭ��ͼ������ȷ��д��
 * ���磬��ԭ��ͼ����LED1�ƽ���stm32f103оƬ��PB0�����ϣ������й�LED1�ĺ궨��
 * ������GPIOB��GPIO_Pin_0��صģ�����ר�Ű���Щ�뿪����Ӳ����ص����ݶ���Ϊ�꣬
 * ������޸Ļ�����ֲ����ǳ����㡣
 */
#define LED1_RCC_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()  //wt.edit
#define LED1_GPIO_PIN                 GPIO_PIN_11
#define LED1_GPIO                     GPIOB


#define LED2_RCC_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()  //wt.edit
#define LED2_GPIO_PIN                 GPIO_PIN_12
#define LED2_GPIO                     GPIOB

#define LED1_OFF                       HAL_GPIO_WritePin(LED1_GPIO,LED1_GPIO_PIN,GPIO_PIN_SET)    // ����ߵ�ƽ
#define LED1_ON                      HAL_GPIO_WritePin(LED1_GPIO,LED1_GPIO_PIN,GPIO_PIN_RESET)  // ����͵�ƽ
#define LED1_TOGGLE                   HAL_GPIO_TogglePin(LED1_GPIO,LED1_GPIO_PIN)                // �����ת

#define LED2_OFF                       HAL_GPIO_WritePin(LED2_GPIO,LED2_GPIO_PIN,GPIO_PIN_SET)    // ����ߵ�ƽ
#define LED2_ON                      HAL_GPIO_WritePin(LED2_GPIO,LED2_GPIO_PIN,GPIO_PIN_RESET)  // ����͵�ƽ
#define LED2_TOGGLE                   HAL_GPIO_TogglePin(LED2_GPIO,LED2_GPIO_PIN)                // �����ת

#define GPIO_PB8_RCC_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()  //wt.edit
#define GPIO_PB8_PIN                   GPIO_PIN_8
#define GPIO_PB8                       GPIOB  

#define GPIO_PB5_RCC_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()  //wt.edit
#define GPIO_PB5_PIN                   GPIO_PIN_5
#define GPIO_PB5                       GPIOB

#define GPIO_PB5_HIGH                   HAL_GPIO_WritePin(GPIO_PB5,GPIO_PB5_PIN ,GPIO_PIN_SET)    // ����ߵ�ƽ
#define GPIO_PB5_LOW                    HAL_GPIO_WritePin(GPIO_PB5,GPIO_PB5_PIN ,GPIO_PIN_RESET)  // ����͵�ƽ

#define PB8_EXTI_IRQn                EXTI9_5_IRQn   
#define PB8_EXTI_IRQHandler          EXTI0_IRQHandler
/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/
void LED_GPIO_Init(void);
void LEDx_StateSet(uint8_t LEDx,LEDState_TypeDef state);

#endif  // __BSP_LED_H__


