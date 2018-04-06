/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "led/bsp_led.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����LED��IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_led.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void LED_GPIO_Init(void)
{
   /* ����IOӲ����ʼ���ṹ����� */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* ʹ��(����)LED���Ŷ�ӦIO�˿�ʱ�� */  
  LED1_RCC_CLK_ENABLE();
  LED2_RCC_CLK_ENABLE();
 
  
  /* ����LED1���������ѹ */
  HAL_GPIO_WritePin(LED1_GPIO, LED1_GPIO_PIN, GPIO_PIN_RESET);

  /* ����LED2���������ѹ */
  HAL_GPIO_WritePin(LED2_GPIO, LED2_GPIO_PIN, GPIO_PIN_RESET);
  /* ����LED3���������ѹ */
 
  
  /* �趨LED1��Ӧ����IO��� */
  GPIO_InitStruct.Pin = LED1_GPIO_PIN;
  /* �趨LED1��Ӧ����IOΪ���ģʽ */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* �趨LED1��Ӧ����IO�����ٶ� */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* ��ʼ��LED1��Ӧ����IO */
  HAL_GPIO_Init(LED1_GPIO, &GPIO_InitStruct);
  
  /* �趨LED2��Ӧ����IO��� */
  GPIO_InitStruct.Pin = LED2_GPIO_PIN;
  /* �趨LED2��Ӧ����IOΪ���ģʽ */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* �趨LED2��Ӧ����IO�����ٶ� */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* ��ʼ��LED2��Ӧ����IO */
  HAL_GPIO_Init(LED2_GPIO, &GPIO_InitStruct);
	
	
	/* �趨PB8��Ӧ����IO��� */
  GPIO_InitStruct.Pin = GPIO_PB8_PIN;
  /* �趨LED2��Ӧ����IOΪ���ģʽ */
  GPIO_InitStruct.Mode =GPIO_MODE_INPUT; //GPIO_MODE_IT_FALLING;//GPIO_MODE_INPUT;//GPIO_MODE_IT_FALLING;//GPIO_MODE_INPUT  ;//GPIO_MODE_ANALOG;
  /* �趨LED2��Ӧ����IO�����ٶ� */
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  /* ��ʼ��LED2��Ӧ����IO */
  HAL_GPIO_Init(GPIO_PB8, &GPIO_InitStruct);
	
	/* �趨PB5��Ӧ����IO��� */
  GPIO_InitStruct.Pin = GPIO_PB5_PIN;
  /* �趨LED2��Ӧ����IOΪ���ģʽ */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;//GPIO_MODE_IT_FALLING;//GPIO_MODE_INPUT  ;//GPIO_MODE_ANALOG;
  /* �趨LED2��Ӧ����IO�����ٶ� */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* ��ʼ��LED2��Ӧ����IO */
  HAL_GPIO_Init(GPIO_PB5, &GPIO_InitStruct);
	GPIO_PB5_HIGH;
  
 // HAL_NVIC_SetPriority(PB8_EXTI_IRQn, 1, 1); //ֹͣ����
 // HAL_NVIC_EnableIRQ(PB8_EXTI_IRQn);

  
}

/**
  * ��������: ���ð���LED�Ƶ�״̬
  * �������: LEDx:����x������Ϊ(1,2,3)����ѡ���Ӧ��LED��
  * ���������state:����LED�Ƶ����״̬��
  *             ��ѡֵ��LED_OFF��LED����
  *                     LED_ON�� LED������
  *                     LED_TOGGLE����תLED
  * �� �� ֵ: ��
  * ˵    �����ú���ʹ�����Ʊ�׼�⺯���ı�̷�������������׼�⺯�����˼�롣
  */
void LEDx_StateSet(uint8_t LEDx,LEDState_TypeDef state)
{
  /* �����������Ƿ�Ϸ� */
  assert_param(IS_LED_TYPEDEF(LEDx));
  assert_param(IS_LED_STATE(state));
  
  /* �ж����õ�LED��״̬���������ΪLED���� */
  if(state==LED_OFF)
  {
    if(LEDx & LED1)            
      LED1_OFF;/* LED1�� */
    
    if(LEDx & LED2)
      LED2_OFF;/* LED2�� */
    
    
  }
  else if(state==LED_ON) /* ����LED��Ϊ�� */
  {
    if(LEDx & LED1)
      LED1_ON;/* LED1�� */
    
    if(LEDx & LED2)
      LED2_ON;/* LED2�� */
    
  
  }
  else
  {
    if(LEDx & LED1)
      LED1_TOGGLE;/* �������������ת */
    
    if(LEDx & LED2)
      LED2_TOGGLE;/* �������������ת */
    
    
  }
}

/******************* *****END OF FILE****/
