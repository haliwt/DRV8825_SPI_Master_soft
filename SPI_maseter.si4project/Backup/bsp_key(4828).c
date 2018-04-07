/**
  ******************************************************************************
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "key/bsp_key.h"
#include "led/bsp_led.h"
#include "DRV8825/drv8825.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ���ذ���IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_key.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
__IO uint8_t stop_key_flag=0;
void KEY_GPIO_Init(void)
{
   /* ����IOӲ����ʼ���ṹ����� */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* ʹ��(����)KEY���Ŷ�ӦIO�˿�ʱ�� */  
  KEY1_RCC_CLK_ENABLE();
  KEY2_RCC_CLK_ENABLE();
  KEY3_RCC_CLK_ENABLE();
	
   /* ����KEY1 GPIO:��������ģʽ */
  GPIO_InitStruct.Pin = KEY1_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO, &GPIO_InitStruct); 
	
  
  /* ����KEY2 GPIO:��������ģʽ */
  GPIO_InitStruct.Pin = KEY2_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO, &GPIO_InitStruct);
  
    /* EXTI interrupt init*/
	/* ����KEY3 GPIO:STOP KEY */
  GPIO_InitStruct.Pin = KEY3_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING ;//GPIO_MODE_IT_RISING_FALLING  ;  
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY3_GPIO, &GPIO_InitStruct);  
  
  HAL_NVIC_SetPriority(KEY3_EXTI_IRQn, 1, 0); //ֹͣ����
  HAL_NVIC_EnableIRQ(KEY3_EXTI_IRQn);
	
	//HAL_NVIC_SetPriority(KEY2_EXTI_IRQn, 2, 1);
  //HAL_NVIC_EnableIRQ(KEY2_EXTI_IRQn);
	
	//HAL_NVIC_SetPriority(KEY1_EXTI_IRQn, 2, 2);
 // HAL_NVIC_EnableIRQ(KEY1_EXTI_IRQn);

  
}

/**
  * ��������: ��ȡ����KEY1��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY1_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(20);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
      while(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL);
       /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

/**
  * ��������: ��ȡ����KEY2��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY2_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(20);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
      while(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL);
       /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}



/**
  * ��������: ��ȡ����KEY3��״̬
  * �����������
  * �� �� ֵ: KEY_DOWN�����������£�
  *           KEY_UP  ������û������
  * ˵    �����ޡ�
  */
KEYState_TypeDef KEY3_StateRead(void)
{
   /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(20);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
       /* ����ɨ����ϣ�ȷ�����������£����ذ���������״̬ */
      return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  return KEY_UP;
}

//�жϷ�����
#if 1
void EXTI0_IRQHandler(void)
{
 
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);   //�����жϴ����ú��� KEY3
  // __HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);   
}
#endif 
/**********************************************
*
*�������ܣ��ⲿ�����жϻص��������жϺ���
*�������жϰ�����GPIO�ڣ�
*
************************************************/
#if 1  //wt.edit 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
            HAL_Delay(10);
            if(KEY3_StateRead()==KEY_DOWN)  	//����LED0��ת
            {
					 
					__HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
                    DRV8825_StopMove(); 
                
              				
            }
		
         
						
 }

#endif


