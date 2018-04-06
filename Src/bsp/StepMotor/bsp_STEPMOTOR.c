
/* 包含头文件 ----------------------------------------------------------------*/
#include "StepMotor/bsp_STEPMOTOR.h" 
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "i2c/bsp_EEPROM.h"
#include "DRV8825/drv8825.h"
#include "usart/bsp_usartx.h"
#include "hextodec/hextodec.h"
#include "led/bsp_led.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;
__IO uint16_t Toggle_Pulse=500;         // 比较输出周期，值越小输出频率越快
__IO uint32_t  home_position=0   ;               //原点位置，wt.edit 2018.01.16
speedRampData srd               = {STOP,CW,0,0};         // 加减速曲线变量
static __IO int32_t  step_position     = 0;           // 当前位置
__IO int32_t  LimPosi = FALSE ; //正方向极限标志位  True:到达极限位  False:未到达极限位
__IO int32_t  LimNega = FALSE ; //负方向极限标志位
__IO uint32_t Origin_Position=0; //马达原点位置设置值 wt.edit
__IO uint32_t PulseNumbers;    //记录脉冲数
uint8_t I2c_Buf_Write[256]={0};
uint8_t I2c_Buf_Read[256]={0};
extern __IO uint8_t stop_flag;
__IO uint32_t step_count = 0;  //
__IO uint32_t ABS_Distance;     //绝对位置坐标，读取EERPOM的值 wt.edit 2018.02.03
__IO uint8_t save_flag;
extern __IO uint8_t NewOrigin_flag;
extern __IO uint8_t PB8_flag;


/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 驱动器相关GPIO初始化配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static void STEPMOTOR_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* 引脚端口时钟使能 */
  STEPMOTOR_TIM_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA_GPIO_CLK_ENABLE();
	STEPMOTOR_SLEEP_GPIO_CLK_ENABLE();
  
  /* 驱动器脉冲控制引脚IO初始化 */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL_PORT, &GPIO_InitStruct);
  
  /* 驱动器方向控制引脚IO初始化 */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_DIR_PORT, &GPIO_InitStruct);
  
  /* 驱动器脱机使能控制引脚IO初始化 */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_ENA_PORT, &GPIO_InitStruct);
	
	/* SLEEP驱动器脱机使能控制引脚IO初始化 */
  GPIO_InitStruct.Pin = STEPMOTOR_SLEEP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_SLEEP_PORT, &GPIO_InitStruct);
  
 
  DRV8825_OUTPUT_DISABLE();
}

/**
  * 函数功能: 驱动器定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void STEPMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // 定时器时钟
  TIM_MasterConfigTypeDef sMasterConfig;                 // 定时器主模式配置
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;   // 刹车和死区时间配置
  TIM_OC_InitTypeDef sConfigOC;                          // 定时器通道比较输出
  
  /* 定时器基本环境配置 */
  htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                                 // 定时器编号
  htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;                  // 定时器预分频器
  htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;                  // 计数方向：向上计数
  htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;                        // 定时器周期
  htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // 时钟分频
  htimx_STEPMOTOR.Init.RepetitionCounter = STEPMOTOR_TIM_REPETITIONCOUNTER;  // 重复计数器
  HAL_TIM_Base_Init(&htimx_STEPMOTOR);

  /* 定时器时钟源配置 */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // 使用内部时钟源
  HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

  /* 初始化定时器比较输出环境 */
  HAL_TIM_OC_Init(&htimx_STEPMOTOR);
  
  /* 定时器主输出模式 */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_STEPMOTOR, &sMasterConfig);
  
  /* 刹车和死区时间配置 */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_STEPMOTOR, &sBreakDeadTimeConfig);

  /* 定时器比较输出配置 */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // 比较输出模式：反转输出
  sConfigOC.Pulse = Toggle_Pulse;                      // 脉冲数,占空比
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;          // 输出极性
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // 互补通道输出极性
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // 快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // 空闲电平
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // 互补通道空闲电平
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_TIM_CHANNEL_x);
	
	

  /* STEPMOTOR相关GPIO初始化配置 */
  STEPMOTOR_GPIO_Init();
	
		HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
		HAL_TIM_OC_Stop(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_DISABLE);//TIM_CCx_ENABLE);// 使能定时器通道
  /* 配置定时器中断优先级并使能 */
  HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);

}

/*****************************************************************
  * 函数功能: 相对位置运动：运动给定的步数
  * 输入参数: step：移动的步数 (正数为顺时针，负数为逆时针).
  *           speed  最大速度,实际值为speed*0.1*rad/sec
  * 返 回 值: 无
  * 说    明: 以给定的步数移动步进电机，使得整个运动距离为
  *           指定的步数.
  **************************************************************/
void STEPMOTOR_AxisMoveRel( int32_t step, uint32_t speed)
{  
   uint32_t tim_count;//获取定时器的计数值
   uint16_t i=8000;
  if(step < 0)      // 步数为负数
  {
	      
		srd.dir = CCW; // 逆时针方向旋转,退步，向马达方向移动！--向后走，退步。
	    STEPMOTOR_DIR_REVERSAL();
		//LimNega=step; //wt.edit 2018.01.16
		#if 1//解决 buf 解回原点的，位置错误
		if((stop_flag==1) && (NewOrigin_flag==0))  //按stop键，执行此语句
		{
		  stop_flag=10;  //执行定时器中断函数
		 // back_flag=0;
		  step =-step; 
		  LimNega=step;
		 // temp_value=-step+DRV8825_Read_CurrentPosition();
		  //step_position=DRV8825_Read_CurrentPosition();
		 // step_count=temp_value;
		  srd.rel_step  =step;
		  //PulseNumbers=DRV8825_Read_CurrentPosition();
		  PulseNumbers= - DRV8825_Read_CurrentPosition(); //wt.edit 18.03.08
		}
		else 
		#endif	
		{
		   step =-step;   // 获取步数绝对值
		   LimNega=step; //wt.edit 2018.01.16
		
		}
		
  }
  else
  {     
	    srd.dir = CW;  // 顺时针方向旋转---向前走，背离马达方向移动。
	    STEPMOTOR_DIR_FORWARD();
		LimPosi=step; 
		#if 1            //stop 按键停顿
		if((stop_flag==1) && (NewOrigin_flag==0)) 
		{
			stop_flag=10;
			LimPosi=step;
			//temp_value=step-DRV8825_Read_CurrentPosition();
			//step_position=DRV8825_Read_CurrentPosition();
			PulseNumbers=DRV8825_Read_CurrentPosition();
			
			srd.rel_step  = step;
			//Display_EEPROM_Value();
		}
		else
		#endif 
	    {
			LimPosi=step;    //wt.edit 2018.01.16 
			srd.rel_step  = step;
		}
    	
  }
	
 // srd.rel_step  = step;	//记录相对运动的步数
  

  if(step != 0)    // 如果目标运动步数不为0
  {
    
    
    #if 1	
			
	do
		{
           // Toggle_Pulse=1200;
			srd.min_delay=3000;
		    srd.run_state = RUN;

			tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
			//比较值，CCRx=tim_count+srd.min_delay
		     __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay); // 设置定时器比较值
			HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// 使能定时器通道 
		    DRV8825_OUTPUT_ENABLE();
    	}
	while(i--);
	#endif
	Toggle_Pulse=DRV8825_ReadSpeed();
	srd.min_delay=Toggle_Pulse;
    srd.run_state = RUN;
	
  }
  tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
	//比较值，CCRx=tim_count+srd.min_delay
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay); // 设置定时器比较值
	HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
	TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// 使能定时器通道 
  DRV8825_OUTPUT_ENABLE();
}

/***********************************************************************
  * 函数功能: 绝对位置移动:运动到给定的位置
  * 输入参数: targert_step: 目标位置的坐标
  *           speed : 移动的速度
  * 返 回 值: 无
  * 说    明: 电机以给定的速度移动到指定位置坐标.命令：0xb0
  *
 **********************************************************************/
void STEPMOTOR_AxisMoveAbs( int32_t targert_step, uint32_t speed)
{
	 int32_t rel_step = 0;
	 int8_t dir = -1;
	 rel_step = step_position - targert_step ; 	//获取当前位置和目标位置之间的步数值
	if(stop_flag==0)
	{
	  stop_flag=21;
	  rel_step=DRV8825_Read_CurrentPosition();   //负数--远离马达方向移动
	}
	else if(rel_step == 0)	
	{
		dir = 0;
	}
	else
		dir = -1;
	//STEPMOTOR_AxisMoveRel(dir*rel_step,speed); 
	DRV8825_Back_AxisMoveRel(dir*rel_step,speed);  //wt.edit 2018.03.11
}


/**************************************************
  * 函数功能: 轴回原点函数
  * 输入参数: 原点位置，
  * 返 回 值: 无
  * 说    明: 以逆时针方向为回原点方向一直旋转,检测到
  *           逆时针方向上的极限后停止,记录当前位置,
  *           作为原点
**************************************************/

void STEPMOTOR_AxisHome(uint16_t speed)
{
	 int32_t rel_step = 0;
	 int8_t dir = -1;
	 home_position = Read_Origin_Position();
     rel_step = step_position - home_position ; 	//获取当前位置和目标位置之间的步数值
	
	if(rel_step == 0)	
	{
		dir = 0;
		PulseNumbers=0;  //wt.18.03.06
		step_position=0; ////wt.18.03.06
	}
	else dir = -1;
	STEPMOTOR_AxisMoveRel(dir*rel_step,speed);
}

/******************************************************************************
  *
  * 函数功能: 绝对位置移动
  *        运动到给定的位置,以坐标原点为参照点
  * 输入参数: targert_step: 目标位置的坐标
  *           speed : 移动的速度
  * 返 回 值: 无
  * 说    明: 电机以给定的速度移动到指定位置坐标. 0x33
  *
*******************************************************************************/
void STEPMOTOR_PC_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed)
{
     int32_t rel_step = 0;
	 int8_t dir = -1;
	 uint32_t temp1,temp2,temp3,temp4;
	
	 temp1=Hex2oct_MD1(abs_high);
	 temp2=Hex2oct_MD2(abs_mid);
	 temp3=Hex2oct_LSB(abs_low);
		
	 ABS_Distance=temp1+temp2+temp3;
	#if 1
	if(stop_flag==0)    //第一次开机，执行此语句
	{
	  
		stop_flag=5;
	  temp4=DRV8825_Read_CurrentPosition();
	  step_count=temp4;
	  rel_step=step_position+step_count-ABS_Distance;    //背离马达方向移动，选此项
	  //rel_step=ABS_Distance-step_position-step_count;	 //往马达方向移动,选此项
	 // Display_EEPROM_Value();
	  PulseNumbers=0;
	  step_position=temp4;
	
	}
	else if(NewOrigin_flag==1)
	{
	  NewOrigin_flag=0;
		step_position=0;
		PulseNumbers=0;
		step_count=0;
	  rel_step=step_position-ABS_Distance;	//wt.edit 2018.01.16
	}
	//home_position = Read_Origin_Position();    //wt.edit 2018.01.16
    //rel_step=home_position-targert_step;
	#endif
	else
   rel_step=step_position-ABS_Distance;	//wt.edit 2018.01.16
	if(rel_step == 0)	
	{
		dir = 0;
	}
	else 
	dir = -1;
	
	STEPMOTOR_AxisMoveRel(dir*rel_step,speed); 
	
    
}


/**************************************************************************************/

/**
  * 函数功能: 定时器中断服务函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 实现加减速过程 
  * TIM1_CC_IRQHandler
  *
***************************************************/
void STEPMOTOR_TIMx_IRQHandler(void)//定时器中断处理
{ 
   uint32_t tim_count=0;
  // 总移动步数计数器
  // static uint32_t 
	//uint32_t step_count = 0;
  //定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
   static uint8_t i=0;
  // uint8_t   flags[]={0x00};

  
  if(__HAL_TIM_GET_IT_SOURCE(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx) !=RESET)
  {
    // 清楚定时器中断
    __HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
    
    // 设置比较值
    tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
   // __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay);
		__HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+Toggle_Pulse); //wt.edit
		

    i++;     // 定时器中断次数计数值
    if(i==2) // 2次，说明已经输出一个完整脉冲
	 {
      i=0;   // 清零定时器中断次数计数值
      
      //在这里判断是否处于边缘运动
      if(srd.dir == CCW)      
      {
        if((LimNega-PulseNumbers==0)||(LimNega<PulseNumbers))
		  {
            srd.run_state = STOP; //停止运动
           // step_position=0;
          }
        else 
          {
           // PulseNumbers++;// save_flag=1;
            
          }
      }
      else 
      {
       if((LimPosi-PulseNumbers==0)||(LimPosi<PulseNumbers))
        {
          srd.run_state = STOP;
        //  step_position=0;
        }
        else 
        {
        // PulseNumbers++;// save_flag=1;
		}
      }
      switch(srd.run_state) // 运行状态
      {
        case STOP:
		   //step_count = 0;  // 清零步数计数器 wt.edit 2018.02.05
		  //step_count=PulseNumbers;  //wt.edit
		   PulseNumbers=0;
		  // save_flag=0;
          // 关闭通道
		  HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x); //wt.edit 
          TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);        
          __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
          DRV8825_OUTPUT_DISABLE(); 
		if(stop_flag==21)
		{
			stop_flag=100;
			PulseNumbers=0;
            step_count=0;
	        step_position=0;  //wt.edit 18.03.07
		}
		 break;

        case RUN:
		 // step_count++;
		  if(stop_flag==10)
		  {
		   stop_flag=20;
		  // PulseNumbers=0;
		  }
		  else
		  PulseNumbers++;
          if(srd.dir==CW)
          {	  	
            step_position++; // 绝对位置加1
            step_count++;
			
          }
          else        //dir=CCW 向马达方向移动，后退
          {
            step_position--; // 绝对位置减1
            step_count--;
			
          }
		 //if(step_count == srd.rel_step)		  //相对运动走过的步数
		 if(PulseNumbers==srd.rel_step) //&& (PulseNumbers == step_position))//wt.edti 01.21
		  {
			srd.run_state = STOP;
			srd.rel_step = 0;		//相对步数清零
			
		}
		break;
		default :break;
	  }      
    }
  }
}

/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* 基本定时器外设时钟使能 */
    STEPMOTOR_TIM_RCC_CLK_ENABLE();
  }
  if(htim_base->Instance==GENERAL_TIMx)
  {
    /* 基本定时器外设时钟使能 */
    GENERAL_TIM_RCC_CLK_ENABLE();
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

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    STEPMOTOR_TIM_RCC_CLK_DISABLE();
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL_PORT,STEPMOTOR_TIM_PUL_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_ENA_PORT,STEPMOTOR_ENA_PIN);
    
    HAL_NVIC_DisableIRQ(STEPMOTOR_TIMx_IRQn);
  }
  if(htim_base->Instance==GENERAL_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    GENERAL_TIM_RCC_CLK_DISABLE();
  }

 }


/******************* *****END OF FILE****/
