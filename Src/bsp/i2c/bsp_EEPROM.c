/**
  *****************************************************************************
	******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "i2c/bsp_EEPROM.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: I2C总线位延迟，最快400KHz
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
static void I2C_Delay(void)
{
	uint8_t i;

	/*　
	 	下面的时间是通过逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
	for (i = 0; i < 10; i++);
}

/**
  * 函数功能: CPU发起I2C总线启动信号
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void I2C_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_HIGH();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_LOW();
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
}

/**
  * 函数功能: CPU发起I2C总线停止信号
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void I2C_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_LOW();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_HIGH();
}

/**
  * 函数功能: CPU向I2C总线设备发送8bit数据
  * 输入参数: Byte ： 等待发送的字节
  * 返 回 值: 无
  * 说    明：无
  */
void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
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
			I2C_SDA_HIGH(); // 释放总线
		}
		Byte <<= 1;	/* 左移一个bit */
		I2C_Delay();
	}
}


/**
  * 函数功能: CPU从I2C总线设备读取8bit数据
  * 输入参数: 无
  * 返 回 值: 读到的数据
  * 说    明：无
  */
uint8_t I2C_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
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
  * 函数功能: CPU产生一个时钟，并读取器件的ACK应答信号
  * 输入参数: 无
  * 返 回 值: 返回0表示正确应答，1表示无器件响应
  * 说    明：无
  */
uint8_t I2C_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_HIGH();	/* CPU释放SDA总线 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	I2C_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
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
  * 函数功能: CPU产生一个ACK信号
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void I2C_Ack(void)
{
	I2C_SDA_LOW();	/* CPU驱动SDA = 0 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU产生1个时钟 */
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
	I2C_SDA_HIGH();	/* CPU释放SDA总线 */
}

/**
  * 函数功能: CPU产生1个NACK信号
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void I2C_NAck(void)
{
	I2C_SDA_HIGH();	/* CPU驱动SDA = 1 */
	I2C_Delay();
	I2C_SCL_HIGH();	/* CPU产生1个时钟 */
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();	
}

/**
  * 函数功能: 配置I2C总线的GPIO，采用模拟IO的方式实现
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
static void I2C_InitGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* 打开GPIO时钟 */
	I2C_GPIO_CLK_ENABLE();
	
   __HAL_AFIO_REMAP_SWJ_NONJTRST();  //wt.edit 2018.03.23 
	
  GPIO_InitStruct.Pin = I2C_SCL_AT24C02_PIN|I2C_SDA_AT24C02_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

  /* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
  I2C_Stop();
}

/**
  * 函数功能: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
  * 输入参数: _Address：设备的I2C总线地址
  * 返 回 值: 返回值 0 表示正确， 返回1表示未探测到
  * 说    明：在访问I2C设备前，请先调用 I2C_CheckDevice() 检测I2C设备是否正常，该函数会配置GPIO
  */
uint8_t I2C_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	I2C_InitGPIO();		/* 配置GPIO */	
	I2C_Start();		/* 发送启动信号 */
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	I2C_SendByte(_Address | I2C_WR);
	ucAck = I2C_WaitAck();	/* 检测设备的ACK应答 */
	I2C_Stop();			/* 发送停止信号 */
	return ucAck;
}

/**
  * 函数功能: 判断串行EERPOM是否正常
  * 输入参数: 无
  * 返 回 值: 1 表示正常， 0 表示不正常
  * 说    明：无
  */
uint8_t EEPROM_CheckOk(void)
{
	if(I2C_CheckDevice(EEPROM_DEV_ADDR) == 0)
	{
		return 1;
	}
	else
	{
		/* 失败后，切记发送I2C总线停止信号 */
		I2C_Stop();		
		return 0;
	}
}

/**
  * 函数功能: 从串行EEPROM指定地址处开始读取若干数据
  * 输入参数: ReadBuf : 存放读到的数据的缓冲区指针
  *           Address : 起始地址  
  *           Size : 数据长度，单位为字节
  * 返 回 值:  0 表示失败，1表示成功
  * 说    明：无
  */
uint8_t EEPROM_ReadBytes(uint8_t *ReadBuf, uint16_t Address, uint16_t Size)
{
	uint16_t i;
	
	/* 采用串行EEPROM随即读取指令序列，连续读取若干字节 */
	
	/* 第1步：发起I2C总线启动信号 */
	I2C_Start();	
  
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	I2C_SendByte(EEPROM_DEV_ADDR | I2C_WR);	/* 此处是写指令 */	
  
	/* 第3步：等待ACK */
	if (I2C_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
  
	/* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
	I2C_SendByte((uint8_t)Address);	
  
	/* 第5步：等待ACK */
	if (I2C_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}	
	/* 第6步：重新启动I2C总线。前面的代码的目的向EEPROM传送地址，下面开始读取数据 */
	I2C_Start();	
  
	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	I2C_SendByte(EEPROM_DEV_ADDR | I2C_RD);	/* 此处是读指令 */
	
	/* 第8步：发送ACK */
	if (I2C_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}	
	
	/* 第9步：循环读取数据 */
	for (i = 0; i < Size; i++)
	{
		ReadBuf[i] = I2C_ReadByte();	/* 读1个字节 */
		
		/* 每读完1个字节后，需要发送Ack， 最后一个字节不需要Ack，发Nack */
		if (i != Size - 1)
		{
			I2C_Ack();	/* 中间字节读完后，CPU产生ACK信号(驱动SDA = 0) */
		}
		else
		{
			I2C_NAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		}
	}
	/* 发送I2C总线停止信号 */
	I2C_Stop();
	return 1;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	I2C_Stop();
	return 0;
}

/**
  * 函数功能: 向串行EEPROM指定地址写入若干数据，采用页写操作提高写入效率
  * 输入参数: WriteBuf : 存放带写入的数据的缓冲区指针
  *           Address : 起始地址
  *           Size : 数据长度，单位为字节 
  * 返 回 值:  0 表示失败，1表示成功
  * 说    明：无
  */
uint8_t EEPROM_WriteBytes(uint8_t *WriteBuf, uint16_t Address, uint16_t Size)
{
	uint16_t i,m;
	uint16_t usAddr;
	
	/* 
	 * 写串行EEPROM不像读操作可以连续读取很多字节，每次写操作只能在同一个page。
	 * 对于24xx02，page size = 8
	 * 简单的处理方法为：按字节写操作模式，没写1个字节，都发送地址
	 * 为了提高连续写的效率: 本函数采用page wirte操作。
	 */

	usAddr = Address;	
	for (i = 0; i < Size; i++)
	{
		/* 当发送第1个字节或是页面首地址时，需要重新发起启动信号和地址 */
		if ((i == 0) || (usAddr & (EEPROM_PAGE_SIZE - 1)) == 0)
		{
			/*　第０步：发停止信号，启动内部写操作　*/
			I2C_Stop();
			
			/* 通过检查器件应答的方式，判断内部写操作是否完成, 一般小于 10ms 			
				CLK频率为200KHz时，查询次数为30次左右
			*/
			for (m = 0; m < 1000; m++)
			{				
				/* 第1步：发起I2C总线启动信号 */
				I2C_Start();
				
				/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
				I2C_SendByte(EEPROM_DEV_ADDR | I2C_WR);	/* 此处是写指令 */
				
				/* 第3步：发送一个时钟，判断器件是否正确应答 */
				if (I2C_WaitAck() == 0)
				{
					break;
				}
			}
			if (m  == 1000)
			{
				goto cmd_fail;	/* EEPROM器件写超时 */
			}
		
			/* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
			I2C_SendByte((uint8_t)usAddr);
			
			/* 第5步：等待ACK */
			if (I2C_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM器件无应答 */
			}
		}
	
		/* 第6步：开始写入数据 */
		I2C_SendByte(WriteBuf[i]);
	
		/* 第7步：发送ACK */
		if (I2C_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}

		usAddr++;	/* 地址增1 */		
	}
	
	/* 命令执行成功，发送I2C总线停止信号 */
	I2C_Stop();
	return 1;

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	I2C_Stop();
	return 0;
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
