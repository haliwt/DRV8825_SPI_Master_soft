/**
 丰笛工业智能科技有限公司
 ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "AdvancedTIM/bsp_AdvancedTIM.h" 

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_advanced;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 定时器硬件初始化配置
  * 输入参数: htim：定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被ADVANCED_TIMx_Init函数调用
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim->Instance==ADVANCED_TIMx)
  {  
    /* 定时器通道功能引脚端口时钟使能 */
    ADVANCED_TIM_GPIO_RCC_CLK_ENABLE();
   
    /* 定时器通道1功能引脚IO初始化 */
    GPIO_InitStruct.Pin = ADVANCED_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADVANCED_TIM_CH1_PORT, &GPIO_InitStruct);

   #if 0
    /* 定时器通道2功能引脚IO初始化 */
    GPIO_InitStruct.Pin = ADVANCED_TIM_CH2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADVANCED_TIM_CH2_PORT, &GPIO_InitStruct);
    
    /* 定时器通道3功能引脚IO初始化 */
    GPIO_InitStruct.Pin = ADVANCED_TIM_CH3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADVANCED_TIM_CH3_PORT, &GPIO_InitStruct);
    
    /* 定时器通道4功能引脚IO初始化 */
    GPIO_InitStruct.Pin = ADVANCED_TIM_CH4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADVANCED_TIM_CH4_PORT, &GPIO_InitStruct);
    #endif 
  }
}

/**
  * 函数功能: 基本定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void ADVANCED_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;
  
  htimx_advanced.Instance = ADVANCED_TIMx;
  htimx_advanced.Init.Prescaler = ADVANCED_TIM_PRESCALER;
  htimx_advanced.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx_advanced.Init.Period = ADVANCED_TIM_PERIOD;
  htimx_advanced.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  htimx_advanced.Init.RepetitionCounter = ADVANCED_TIM_REPETITIONCOUNTER;
  HAL_TIM_Base_Init(&htimx_advanced);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htimx_advanced, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htimx_advanced);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_advanced, &sMasterConfig);
  
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_advanced, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 700;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htimx_advanced, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.Pulse = 600;
  HAL_TIM_PWM_ConfigChannel(&htimx_advanced, &sConfigOC, TIM_CHANNEL_2);

  sConfigOC.Pulse = 450;
  HAL_TIM_PWM_ConfigChannel(&htimx_advanced, &sConfigOC, TIM_CHANNEL_3);

  sConfigOC.Pulse = 100;
  HAL_TIM_PWM_ConfigChannel(&htimx_advanced, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htimx_advanced);

}

/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
#if 0
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==ADVANCED_TIMx)
  {
    /* 基本定时器外设时钟使能 */
    ADVANCED_TIM_RCC_CLK_ENABLE();
  }
}

/**
  * 函数功能: 基本定时器硬件反初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==ADVANCED_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    ADVANCED_TIM_RCC_CLK_DISABLE();
  }
} 
#endif

