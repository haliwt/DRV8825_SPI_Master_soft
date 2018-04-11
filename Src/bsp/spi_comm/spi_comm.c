#include "spi_comm/spi_comm.h"
#include "spi/bsp_spi.h"
#include "usart/bsp_usartx.h"
#include "led/bsp_led.h"
#include "DRV8825/drv8825.h"

extern uint8_t repcdata[3];
extern uint8_t aRxBuffer[256];                      // 接收数据 
uint8_t SPI_aRxBuffer[20];
uint8_t SPI_aTxBuffer[7]={0xA2,0x00,0xff,0x00,0x00,0x00,0x0b};
uint8_t test_aTxBuffer[7]={0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0x0b};
uint8_t I2C_RxBuffer[4];
extern __IO uint16_t judge_data;    //接收到数据的功能码数据。
extern __IO uint8_t re_intrrupt_flag; //从上位机，接收信号标志位，进入中断标志位


uint8_t SPI_TX_flag=0;

void SPI_COMM_Function(uint8_t spi_order,uint8_t spi_tx_hig,uint8_t spi_tx_mid,uint8_t spi_tx_low)
{
     
	//  uint8_t i=0;
	//for(i=0;i<2;i++)
	{
	 
	//  SPIx_CS_DISABLE();  
	//  HAL_Delay(10);
	 // SPIx_CS_ENABLE();
	 // HAL_Delay(10);
	  SPI_aTxBuffer[0]=0xa2;
	  SPI_aTxBuffer[2]=spi_order;
	  SPI_aTxBuffer[3]=spi_tx_hig;
	  SPI_aTxBuffer[4]=spi_tx_mid;
	  SPI_aTxBuffer[5]=spi_tx_low;
	  SPI_aTxBuffer[6]=0x0b;
	//HAL_SPI_Transmit(&hspi_SPI,&SPI_aTxBuffer[0],7,0xFFFF);
	if(HAL_SPI_Transmit(&hspi_SPI,&SPI_aTxBuffer[0],7,0xFFF)==HAL_OK)
     {
       SPI_TX_flag=1;
	  // if(i==1)
	   {
	   printf("主机发送消息成功\n");
       printf("aTxBuffer[0]=%#x\n",SPI_aTxBuffer[0]);
	   printf("aTxBuffer[1]=%#x\n",SPI_aTxBuffer[1]);
	   printf("aTxBuffer[2]=%#x\n",SPI_aTxBuffer[2]);
	   printf("aTxBuffer[3]=%#x\n",SPI_aTxBuffer[3]);
	   printf("aTxBuffer[4]=%#x\n",SPI_aTxBuffer[4]);
	   printf("aTxBuffer[5]=%#x\n",SPI_aTxBuffer[5]);
	   printf("aTxBuffer[6]=%#x\n",SPI_aTxBuffer[6]);
	  // SPIx_CS_DISABLE();  
	   HAL_Delay(10);
      
	  }
  }
    }
     
	//else SPI_TX_flag=0;
}
	
    



void SYNC_COMM_TEST(void)
{
    
	
	if(HAL_SPI_Transmit(&hspi_SPI,&test_aTxBuffer[0],7,0xffff)==HAL_OK)
     {
       
	  
	   
	   printf("主机发送消息成功\n");
       printf("aTxBuffer[0]=%#x\n",test_aTxBuffer[0]);
	   printf("aTxBuffer[1]=%#x\n",test_aTxBuffer[1]);
	   printf("aTxBuffer[2]=%#x\n",test_aTxBuffer[2]);
	   printf("aTxBuffer[3]=%#x\n",test_aTxBuffer[3]);
	   printf("aTxBuffer[4]=%#x\n",test_aTxBuffer[4]);
	   printf("aTxBuffer[5]=%#x\n",test_aTxBuffer[5]);
	   printf("aTxBuffer[6]=%#x\n",test_aTxBuffer[6]);
	  
	   
       HAL_Delay(10);
	  }
	  

   }


