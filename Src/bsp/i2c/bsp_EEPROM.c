/**
  *****************************************************************************
	******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "i2c/bsp_EEPROM.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: I2C����λ�ӳ٣����400KHz
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
static void I2C_Delay(void)
{
	uint8_t i;

	/*��
	 	�����ʱ����ͨ���߼������ǲ��Եõ��ġ�
		CPU��Ƶ72MHzʱ�����ڲ�Flash����, MDK���̲��Ż�
		ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
		ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
	 	ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
        
    IAR���̱���Ч�ʸߣ���������Ϊ7
	*/
	for (i = 0; i < 10; i++);
}

/**
  * ��������: CPU����I2C���������ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
	I2C_SDA_HIGH();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_LOW();
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
}

/**
  * ��������: CPU����I2C����ֹͣ�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	I2C_SDA_LOW();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_HIGH();
}

/**
  * ��������: CPU��I2C�����豸����8bit����
  * �������: Byte �� �ȴ����͵��ֽ�
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
	for (i = 0; i < 8; i++)
	{		
		if (Byte & 0x80)
		{
			I2C_SDA_HIGH();
		}
		else
		{
			I2C_SDA_LOW();
		}
		I2C_Delay();
		I2C_SCL_HIGH();
		I2C_Delay();	
		I2C_SCL_LOW();
		if (i == 7)
		{
			I2C_SDA_HIGH(); // �ͷ�����
		}
		Byte <<= 1;	/* ����һ��bit */
		I2C_Delay();
	}
}


/**
  * ��������: CPU��I2C�����豸��ȡ8bit����
  * �������: ��
  * �� �� ֵ: ����������
  * ˵    ������
  */
uint8_t I2C_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_HIGH();
		I2C_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_LOW();
		I2C_Delay();
	}
	return value;
}

/**
  * ��������: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
  * �������: ��
  * �� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
  * ˵    ������
  */
uint8_t I2C_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_HIGH();	/* CPU�ͷ�SDA���� */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	I2C_Delay();
	if (I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_LOW();
	I2C_Delay();
	return re;
}

/**
  * ��������: CPU����һ��ACK�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_Ack(void)
{
	I2C_SDA_LOW();	/* CPU����SDA = 0 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����1��ʱ�� */
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
	I2C_SDA_HIGH();	/* CPU�ͷ�SDA���� */
}

/**
  * ��������: CPU����1��NACK�ź�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void I2C_NAck(void)
{
	I2C_SDA_HIGH();	/* CPU����SDA = 1 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU����1��ʱ�� */
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();	
}

/**
  * ��������: ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
static void I2C_InitGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��GPIOʱ�� */
	I2C_GPIO_CLK_ENABLE();
	
   __HAL_AFIO_REMAP_SWJ_NONJTRST();  //wt.edit 2018.03.23 
	
  GPIO_InitStruct.Pin = I2C_SCL_AT24C02_PIN|I2C_SDA_AT24C02_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

  /* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
  I2C_Stop();
}

/**
  * ��������: ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
  * �������: _Address���豸��I2C���ߵ�ַ
  * �� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
  * ˵    �����ڷ���I2C�豸ǰ�����ȵ��� I2C_CheckDevice() ���I2C�豸�Ƿ��������ú���������GPIO
  */
uint8_t I2C_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	I2C_InitGPIO();		/* ����GPIO */	
	I2C_Start();		/* ���������ź� */
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	I2C_SendByte(_Address | I2C_WR);
	ucAck = I2C_WaitAck();	/* ����豸��ACKӦ�� */
	I2C_Stop();			/* ����ֹͣ�ź� */
	return ucAck;
}

/**
  * ��������: �жϴ���EERPOM�Ƿ�����
  * �������: ��
  * �� �� ֵ: 1 ��ʾ������ 0 ��ʾ������
  * ˵    ������
  */
uint8_t EEPROM_CheckOk(void)
{
	if(I2C_CheckDevice(EEPROM_DEV_ADDR) == 0)
	{
		return 1;
	}
	else
	{
		/* ʧ�ܺ��мǷ���I2C����ֹͣ�ź� */
		I2C_Stop();		
		return 0;
	}
}

/**
  * ��������: �Ӵ���EEPROMָ����ַ����ʼ��ȡ��������
  * �������: ReadBuf : ��Ŷ��������ݵĻ�����ָ��
  *           Address : ��ʼ��ַ  
  *           Size : ���ݳ��ȣ���λΪ�ֽ�
  * �� �� ֵ:  0 ��ʾʧ�ܣ�1��ʾ�ɹ�
  * ˵    ������
  */
uint8_t EEPROM_ReadBytes(uint8_t *ReadBuf, uint16_t Address, uint16_t Size)
{
	uint16_t i;
	
	/* ���ô���EEPROM�漴��ȡָ�����У�������ȡ�����ֽ� */
	
	/* ��1��������I2C���������ź� */
	I2C_Start();	
  
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	I2C_SendByte(EEPROM_DEV_ADDR | I2C_WR);	/* �˴���дָ�� */	
  
	/* ��3�����ȴ�ACK */
	if (I2C_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
  
	/* ��4���������ֽڵ�ַ��24C02ֻ��256�ֽڣ����1���ֽھ͹��ˣ������24C04���ϣ���ô�˴���Ҫ���������ַ */
	I2C_SendByte((uint8_t)Address);	
  
	/* ��5�����ȴ�ACK */
	if (I2C_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}	
	/* ��6������������I2C���ߡ�ǰ��Ĵ����Ŀ����EEPROM���͵�ַ�����濪ʼ��ȡ���� */
	I2C_Start();	
  
	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	I2C_SendByte(EEPROM_DEV_ADDR | I2C_RD);	/* �˴��Ƕ�ָ�� */
	
	/* ��8��������ACK */
	if (I2C_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}	
	
	/* ��9����ѭ����ȡ���� */
	for (i = 0; i < Size; i++)
	{
		ReadBuf[i] = I2C_ReadByte();	/* ��1���ֽ� */
		
		/* ÿ����1���ֽں���Ҫ����Ack�� ���һ���ֽڲ���ҪAck����Nack */
		if (i != Size - 1)
		{
			I2C_Ack();	/* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */
		}
		else
		{
			I2C_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
		}
	}
	/* ����I2C����ֹͣ�ź� */
	I2C_Stop();
	return 1;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	I2C_Stop();
	return 0;
}

/**
  * ��������: ����EEPROMָ����ַд���������ݣ�����ҳд�������д��Ч��
  * �������: WriteBuf : ��Ŵ�д������ݵĻ�����ָ��
  *           Address : ��ʼ��ַ
  *           Size : ���ݳ��ȣ���λΪ�ֽ� 
  * �� �� ֵ:  0 ��ʾʧ�ܣ�1��ʾ�ɹ�
  * ˵    ������
  */
uint8_t EEPROM_WriteBytes(uint8_t *WriteBuf, uint16_t Address, uint16_t Size)
{
	uint16_t i,m;
	uint16_t usAddr;
	
	/* 
	 * д����EEPROM�������������������ȡ�ܶ��ֽڣ�ÿ��д����ֻ����ͬһ��page��
	 * ����24xx02��page size = 8
	 * �򵥵Ĵ�����Ϊ�����ֽ�д����ģʽ��ûд1���ֽڣ������͵�ַ
	 * Ϊ���������д��Ч��: ����������page wirte������
	 */

	usAddr = Address;	
	for (i = 0; i < Size; i++)
	{
		/* �����͵�1���ֽڻ���ҳ���׵�ַʱ����Ҫ���·��������źź͵�ַ */
		if ((i == 0) || (usAddr & (EEPROM_PAGE_SIZE - 1)) == 0)
		{
			/*���ڣ�������ֹͣ�źţ������ڲ�д������*/
			I2C_Stop();
			
			/* ͨ���������Ӧ��ķ�ʽ���ж��ڲ�д�����Ƿ����, һ��С�� 10ms 			
				CLKƵ��Ϊ200KHzʱ����ѯ����Ϊ30������
			*/
			for (m = 0; m < 1000; m++)
			{				
				/* ��1��������I2C���������ź� */
				I2C_Start();
				
				/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
				I2C_SendByte(EEPROM_DEV_ADDR | I2C_WR);	/* �˴���дָ�� */
				
				/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
				if (I2C_WaitAck() == 0)
				{
					break;
				}
			}
			if (m  == 1000)
			{
				goto cmd_fail;	/* EEPROM����д��ʱ */
			}
		
			/* ��4���������ֽڵ�ַ��24C02ֻ��256�ֽڣ����1���ֽھ͹��ˣ������24C04���ϣ���ô�˴���Ҫ���������ַ */
			I2C_SendByte((uint8_t)usAddr);
			
			/* ��5�����ȴ�ACK */
			if (I2C_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM������Ӧ�� */
			}
		}
	
		/* ��6������ʼд������ */
		I2C_SendByte(WriteBuf[i]);
	
		/* ��7��������ACK */
		if (I2C_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}

		usAddr++;	/* ��ַ��1 */		
	}
	
	/* ����ִ�гɹ�������I2C����ֹͣ�ź� */
	I2C_Stop();
	return 1;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	I2C_Stop();
	return 0;
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
