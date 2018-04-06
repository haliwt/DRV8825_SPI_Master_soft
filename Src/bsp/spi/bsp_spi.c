
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "spi/bsp_spi.h"
#include "usart/bsp_usartx.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/                              
/* ˽�б��� ------------------------------------------------------------------*/
SPI_HandleTypeDef hspi_SPI;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����FLASH��ʼ��
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
*/
void SPIx_Init(void)
{
  hspi_SPI.Instance = SPIx;
  hspi_SPI.Init.Mode =  SPI_MODE_MASTER;  //����
  hspi_SPI.Init.Direction = SPI_DIRECTION_2LINES; 
  hspi_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_SPI.Init.CLKPolarity = SPI_POLARITY_LOW; // 2018.03.15 edit
  hspi_SPI.Init.CLKPhase = SPI_PHASE_1EDGE;  //wt.edit 2018.03.13
  hspi_SPI.Init.NSS = SPI_NSS_SOFT;
  hspi_SPI.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi_SPI.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_SPI.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_SPI.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_SPI.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi_SPI);
  __HAL_SPI_ENABLE(&hspi_SPI);
}

/**
  * ��������: SPI����ϵͳ����ʼ��
  * �������: hspi��SPI�������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPIx)
  {
    /* SPI����ʱ��ʹ�� */
    SPIx_RCC_CLK_ENABLE();
    /* GPIO����ʱ��ʹ�� */
    SPIx_GPIO_ClK_ENABLE();
    SPIx_CS_CLK_ENABLE();
    
    /* Disable the Serial Wire Jtag Debug Port SWJ-DP */
    //__HAL_AFIO_REMAP_SWJ_DISABLE();
    
    /**SPI3 GPIO Configuration    
    PF11     ------> SPI3_NSS
    PB3      ------> SPI3_SCK
    PB4      ------> SPI3_MISO
    PB5      ------> SPI3_MOSI 
    */
    GPIO_InitStruct.Pin = SPIx_SCK_PIN|SPIx_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SPIx_CS_PIN;
    //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  //GPIO_MODE_OUTPUT_PP;wt.edit 2018.03.12, �䨮?��
	GPIO_InitStruct.Pull=GPIO_PULLUP;      //wt.edit 2018.03.12
    HAL_GPIO_Init(SPIx_CS_PORT, &GPIO_InitStruct);
    
    
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);    
  }
}

/**
  * ��������: SPI����ϵͳ������ʼ��
  * �������: hspi��SPI�������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPIx)
  {
    /* SPI����ʱ�ӽ��� */
    SPIx_RCC_CLK_DISABLE();
  
    /**SPI3 GPIO Configuration    
    PF11     ------> SPI3_NSS
    PB3      ------> SPI3_SCK
    PB4      ------> SPI3_MISO
    PB5      ------> SPI3_MOSI 
    */
    HAL_GPIO_DeInit(SPIx_GPIO_PORT, SPIx_SCK_PIN|SPIx_MISO_PIN|SPIx_MOSI_PIN);
    HAL_GPIO_DeInit(SPIx_CS_PORT, SPIx_CS_PIN);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  }
} 

#if 0
/**
  * ��������: ������Flash��ȡд��һ���ֽ����ݲ�����һ���ֽ�����
  * �������: byte������������
  * �� �� ֵ: uint8_t�����յ�������
  * ˵    ������
  */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint8_t byte)
{
  uint8_t d_read,d_send=byte;
  if(HAL_SPI_TransmitReceive(hspi,&d_send,&d_read,1,0xFF)!=HAL_OK)
  {   
    d_read=0xFF;
  }
  printf("d_read=%x\n",d_read);
  return d_read; 
}
#endif
//SPI1 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi_SPI,&TxData,&Rxdata,7, 0xff);       
 	return Rxdata;          		    //�����յ�������		
}


