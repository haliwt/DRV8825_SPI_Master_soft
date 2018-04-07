/**
  ******************************************************************************
  ******************************************************************************
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "key/bsp_key.h"
#include "led/bsp_led.h"
#include "DRV8825/drv8825.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 板载按键IO引脚初始化.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：使用宏定义方法代替具体引脚号，方便程序移植，只要简单修改bsp_key.h
  *           文件相关宏定义就可以方便修改引脚。
  */
__IO uint8_t stop_key_flag=0;
void KEY_GPIO_Init(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* 使能(开启)KEY引脚对应IO端口时钟 */  
  KEY1_RCC_CLK_ENABLE();
  KEY2_RCC_CLK_ENABLE();
  KEY3_RCC_CLK_ENABLE();
	
   /* 配置KEY1 GPIO:输入下拉模式 */
  GPIO_InitStruct.Pin = KEY1_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO, &GPIO_InitStruct); 
	
  
  /* 配置KEY2 GPIO:输入上拉模式 */
  GPIO_InitStruct.Pin = KEY2_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO, &GPIO_InitStruct);
  
    /* EXTI interrupt init*/
	/* 配置KEY3 GPIO:STOP KEY */
  GPIO_InitStruct.Pin = KEY3_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING ;//GPIO_MODE_IT_RISING_FALLING  ;  
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY3_GPIO, &GPIO_InitStruct);  
  
  HAL_NVIC_SetPriority(KEY3_EXTI_IRQn, 1, 0); //停止按键
  HAL_NVIC_EnableIRQ(KEY3_EXTI_IRQn);
	
	//HAL_NVIC_SetPriority(KEY2_EXTI_IRQn, 2, 1);
  //HAL_NVIC_EnableIRQ(KEY2_EXTI_IRQn);
	
	//HAL_NVIC_SetPriority(KEY1_EXTI_IRQn, 2, 2);
 // HAL_NVIC_EnableIRQ(KEY1_EXTI_IRQn);

  
}

/**
  * 函数功能: 读取按键KEY1的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY1_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(20);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
      while(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY2的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY2_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(20);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
      while(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}



/**
  * 函数功能: 读取按键KEY3的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY3_StateRead(void)
{
   /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(20);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

//中断服务函数
#if 1
void EXTI0_IRQHandler(void)
{
 
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);   //调用中断处理公用函数 KEY3
  // __HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);   
}
#endif 
/**********************************************
*
*函数功能：外部按键中断回调函数，中断函数
*参数：中断按键的GPIO口，
*
************************************************/
#if 1  //wt.edit 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
            HAL_Delay(10);
            if(KEY3_StateRead()==KEY_DOWN)  	//控制LED0翻转
            {
					 
					__HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
                    DRV8825_StopMove(); 
                
              				
            }
		
         
						
 }

#endif


