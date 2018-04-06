
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "StepMotor/bsp_STEPMOTOR.h" 
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "i2c/bsp_EEPROM.h"
#include "DRV8825/drv8825.h"
#include "usart/bsp_usartx.h"
#include "hextodec/hextodec.h"
#include "led/bsp_led.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;
__IO uint16_t Toggle_Pulse=500;         // �Ƚ�������ڣ�ֵԽС���Ƶ��Խ��
__IO uint32_t  home_position=0   ;               //ԭ��λ�ã�wt.edit 2018.01.16
speedRampData srd               = {STOP,CW,0,0};         // �Ӽ������߱���
static __IO int32_t  step_position     = 0;           // ��ǰλ��
__IO int32_t  LimPosi = FALSE ; //�������ޱ�־λ  True:���Ｋ��λ  False:δ���Ｋ��λ
__IO int32_t  LimNega = FALSE ; //�������ޱ�־λ
__IO uint32_t Origin_Position=0; //���ԭ��λ������ֵ wt.edit
__IO uint32_t PulseNumbers;    //��¼������
uint8_t I2c_Buf_Write[256]={0};
uint8_t I2c_Buf_Read[256]={0};
extern __IO uint8_t stop_flag;
__IO uint32_t step_count = 0;  //
__IO uint32_t ABS_Distance;     //����λ�����꣬��ȡEERPOM��ֵ wt.edit 2018.02.03
__IO uint8_t save_flag;
extern __IO uint8_t NewOrigin_flag;
extern __IO uint8_t PB8_flag;


/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void STEPMOTOR_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* ���Ŷ˿�ʱ��ʹ�� */
  STEPMOTOR_TIM_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA_GPIO_CLK_ENABLE();
	STEPMOTOR_SLEEP_GPIO_CLK_ENABLE();
  
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL_PORT, &GPIO_InitStruct);
  
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_DIR_PORT, &GPIO_InitStruct);
  
  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_ENA_PORT, &GPIO_InitStruct);
	
	/* SLEEP�������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_SLEEP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_SLEEP_PORT, &GPIO_InitStruct);
  
 
  DRV8825_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void STEPMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_MasterConfigTypeDef sMasterConfig;                 // ��ʱ����ģʽ����
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;   // ɲ��������ʱ������
  TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����
  
  /* ��ʱ�������������� */
  htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                                 // ��ʱ�����
  htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;                  // ��ʱ��Ԥ��Ƶ��
  htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;                  // �����������ϼ���
  htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;                        // ��ʱ������
  htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // ʱ�ӷ�Ƶ
  htimx_STEPMOTOR.Init.RepetitionCounter = STEPMOTOR_TIM_REPETITIONCOUNTER;  // �ظ�������
  HAL_TIM_Base_Init(&htimx_STEPMOTOR);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

  /* ��ʼ����ʱ���Ƚ�������� */
  HAL_TIM_OC_Init(&htimx_STEPMOTOR);
  
  /* ��ʱ�������ģʽ */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_STEPMOTOR, &sMasterConfig);
  
  /* ɲ��������ʱ������ */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_STEPMOTOR, &sBreakDeadTimeConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = Toggle_Pulse;                      // ������,ռ�ձ�
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;          // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_TIM_CHANNEL_x);
	
	

  /* STEPMOTOR���GPIO��ʼ������ */
  STEPMOTOR_GPIO_Init();
	
		HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
		HAL_TIM_OC_Stop(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x,TIM_CCx_DISABLE);//TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ��
  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);

}

/*****************************************************************
  * ��������: ���λ���˶����˶������Ĳ���
  * �������: step���ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
  *           speed  ����ٶ�,ʵ��ֵΪspeed*0.1*rad/sec
  * �� �� ֵ: ��
  * ˵    ��: �Ը����Ĳ����ƶ����������ʹ�������˶�����Ϊ
  *           ָ���Ĳ���.
  **************************************************************/
void STEPMOTOR_AxisMoveRel( int32_t step, uint32_t speed)
{  
   uint32_t tim_count;//��ȡ��ʱ���ļ���ֵ
   uint16_t i=8000;
  if(step < 0)      // ����Ϊ����
  {
	      
		srd.dir = CCW; // ��ʱ�뷽����ת,�˲�������﷽���ƶ���--����ߣ��˲���
	    STEPMOTOR_DIR_REVERSAL();
		//LimNega=step; //wt.edit 2018.01.16
		#if 1//��� buf ���ԭ��ģ�λ�ô���
		if((stop_flag==1) && (NewOrigin_flag==0))  //��stop����ִ�д����
		{
		  stop_flag=10;  //ִ�ж�ʱ���жϺ���
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
		   step =-step;   // ��ȡ��������ֵ
		   LimNega=step; //wt.edit 2018.01.16
		
		}
		
  }
  else
  {     
	    srd.dir = CW;  // ˳ʱ�뷽����ת---��ǰ�ߣ�������﷽���ƶ���
	    STEPMOTOR_DIR_FORWARD();
		LimPosi=step; 
		#if 1            //stop ����ͣ��
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
	
 // srd.rel_step  = step;	//��¼����˶��Ĳ���
  

  if(step != 0)    // ���Ŀ���˶�������Ϊ0
  {
    
    
    #if 1	
			
	do
		{
           // Toggle_Pulse=1200;
			srd.min_delay=3000;
		    srd.run_state = RUN;

			tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
			//�Ƚ�ֵ��CCRx=tim_count+srd.min_delay
		     __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay); // ���ö�ʱ���Ƚ�ֵ
			HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ�� 
		    DRV8825_OUTPUT_ENABLE();
    	}
	while(i--);
	#endif
	Toggle_Pulse=DRV8825_ReadSpeed();
	srd.min_delay=Toggle_Pulse;
    srd.run_state = RUN;
	
  }
  tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
	//�Ƚ�ֵ��CCRx=tim_count+srd.min_delay
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay); // ���ö�ʱ���Ƚ�ֵ
	HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
	TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ�� 
  DRV8825_OUTPUT_ENABLE();
}

/***********************************************************************
  * ��������: ����λ���ƶ�:�˶���������λ��
  * �������: targert_step: Ŀ��λ�õ�����
  *           speed : �ƶ����ٶ�
  * �� �� ֵ: ��
  * ˵    ��: ����Ը������ٶ��ƶ���ָ��λ������.���0xb0
  *
 **********************************************************************/
void STEPMOTOR_AxisMoveAbs( int32_t targert_step, uint32_t speed)
{
	 int32_t rel_step = 0;
	 int8_t dir = -1;
	 rel_step = step_position - targert_step ; 	//��ȡ��ǰλ�ú�Ŀ��λ��֮��Ĳ���ֵ
	if(stop_flag==0)
	{
	  stop_flag=21;
	  rel_step=DRV8825_Read_CurrentPosition();   //����--Զ����﷽���ƶ�
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
  * ��������: ���ԭ�㺯��
  * �������: ԭ��λ�ã�
  * �� �� ֵ: ��
  * ˵    ��: ����ʱ�뷽��Ϊ��ԭ�㷽��һֱ��ת,��⵽
  *           ��ʱ�뷽���ϵļ��޺�ֹͣ,��¼��ǰλ��,
  *           ��Ϊԭ��
**************************************************/

void STEPMOTOR_AxisHome(uint16_t speed)
{
	 int32_t rel_step = 0;
	 int8_t dir = -1;
	 home_position = Read_Origin_Position();
     rel_step = step_position - home_position ; 	//��ȡ��ǰλ�ú�Ŀ��λ��֮��Ĳ���ֵ
	
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
  * ��������: ����λ���ƶ�
  *        �˶���������λ��,������ԭ��Ϊ���յ�
  * �������: targert_step: Ŀ��λ�õ�����
  *           speed : �ƶ����ٶ�
  * �� �� ֵ: ��
  * ˵    ��: ����Ը������ٶ��ƶ���ָ��λ������. 0x33
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
	if(stop_flag==0)    //��һ�ο�����ִ�д����
	{
	  
		stop_flag=5;
	  temp4=DRV8825_Read_CurrentPosition();
	  step_count=temp4;
	  rel_step=step_position+step_count-ABS_Distance;    //������﷽���ƶ���ѡ����
	  //rel_step=ABS_Distance-step_position-step_count;	 //����﷽���ƶ�,ѡ����
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
  * ��������: ��ʱ���жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ��� 
  * TIM1_CC_IRQHandler
  *
***************************************************/
void STEPMOTOR_TIMx_IRQHandler(void)//��ʱ���жϴ���
{ 
   uint32_t tim_count=0;
  // ���ƶ�����������
  // static uint32_t 
	//uint32_t step_count = 0;
  //��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
   static uint8_t i=0;
  // uint8_t   flags[]={0x00};

  
  if(__HAL_TIM_GET_IT_SOURCE(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx) !=RESET)
  {
    // �����ʱ���ж�
    __HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
    
    // ���ñȽ�ֵ
    tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
   // __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay);
		__HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+Toggle_Pulse); //wt.edit
		

    i++;     // ��ʱ���жϴ�������ֵ
    if(i==2) // 2�Σ�˵���Ѿ����һ����������
	 {
      i=0;   // ���㶨ʱ���жϴ�������ֵ
      
      //�������ж��Ƿ��ڱ�Ե�˶�
      if(srd.dir == CCW)      
      {
        if((LimNega-PulseNumbers==0)||(LimNega<PulseNumbers))
		  {
            srd.run_state = STOP; //ֹͣ�˶�
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
      switch(srd.run_state) // ����״̬
      {
        case STOP:
		   //step_count = 0;  // ���㲽�������� wt.edit 2018.02.05
		  //step_count=PulseNumbers;  //wt.edit
		   PulseNumbers=0;
		  // save_flag=0;
          // �ر�ͨ��
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
            step_position++; // ����λ�ü�1
            step_count++;
			
          }
          else        //dir=CCW ����﷽���ƶ�������
          {
            step_position--; // ����λ�ü�1
            step_count--;
			
          }
		 //if(step_count == srd.rel_step)		  //����˶��߹��Ĳ���
		 if(PulseNumbers==srd.rel_step) //&& (PulseNumbers == step_position))//wt.edti 01.21
		  {
			srd.run_state = STOP;
			srd.rel_step = 0;		//��Բ�������
			
		}
		break;
		default :break;
	  }      
    }
  }
}

/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* ������ʱ������ʱ��ʹ�� */
    STEPMOTOR_TIM_RCC_CLK_ENABLE();
  }
  if(htim_base->Instance==GENERAL_TIMx)
  {
    /* ������ʱ������ʱ��ʹ�� */
    GENERAL_TIM_RCC_CLK_ENABLE();
  }
}

/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    STEPMOTOR_TIM_RCC_CLK_DISABLE();
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL_PORT,STEPMOTOR_TIM_PUL_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_ENA_PORT,STEPMOTOR_ENA_PIN);
    
    HAL_NVIC_DisableIRQ(STEPMOTOR_TIMx_IRQn);
  }
  if(htim_base->Instance==GENERAL_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    GENERAL_TIM_RCC_CLK_DISABLE();
  }

 }


/******************* *****END OF FILE****/
