/*******************************************
*
*I2C ���ӻ�ͨ�ţ��ӻ�ͨ��I2C����
*
*
***********************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "i2c_slave/bsp_I2C.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
I2C_HandleTypeDef I2cHandle;
extern uint8_t i2c_rx_data;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: I2C�����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_I2C_EEPROM_Init(void)
{
  I2cHandle.Instance             = I2Cx;
  I2cHandle.Init.ClockSpeed      = I2C_SPEEDCLOCK;
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE;
  I2cHandle.Init.OwnAddress1     = 0x30F;
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2     = 0xFF;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&I2cHandle);
}

/**
  * ��������: I2C����Ӳ����ʼ������
  * �������: hi2c��I2C�������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ʹ������ʱ�� */
  I2C_RCC_CLK_ENABLE();        
  I2C_GPIO_CLK_ENABLE();
  
  /**I2C1 GPIO Configuration    
  PB6     ------> I2C1_SCL
  PB7     ------> I2C1_SDA 
  */
  GPIO_InitStruct.Pin = I2C_SCL_PIN|I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
}

/**
  * ��������: I2C����Ӳ������ʼ������
  * �������: hi2c��I2C�������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  /* ��������ʱ�� */
  I2C_GPIO_CLK_DISABLE();

  /**I2C1 GPIO Configuration    
  PB6     ------> I2C1_SCL
  PB7     ------> I2C1_SDA 
  */
  HAL_GPIO_DeInit(I2C_GPIO_PORT, I2C_SCL_PIN|I2C_SDA_PIN);
  HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);

}

/*****************************************************
*
*
*
*
****************************************************/
void I2C_SLAVE_RX_FUN(void)
{ 
     
    HAL_I2C_Slave_Receive_IT(&I2cHandle,&i2c_rx_data,4);
	    
}



/********************���************************/
