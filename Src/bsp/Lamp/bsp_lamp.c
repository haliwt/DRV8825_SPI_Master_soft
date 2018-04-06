#include "lamp/bsp_lamp.h"
#include "i2c/bsp_EEPROM.h"
#include "led/bsp_led.h"
#include "usart/bsp_usartx.h"

extern uint8_t I2c_Buf_Write[256];
extern uint8_t I2c_Buf_Read[256];
extern __IO uint8_t Brightness;
//extern uint8_t I2C_RX_Buffer[3];  //I2C ���յ�������
extern uint8_t I2C_RX_SAVE_Buffer[3];
void LAMP_Save_BrightValue(uint8_t brig_v)
{

      uint8_t flag,re;
	  flag=EEPROM_CheckOk();
	 if(flag==1)
	 {
    
      I2c_Buf_Read[0]=0;      // ��ս��ջ�����
      I2c_Buf_Write[0] =brig_v ;   // Ϊ���ͻ������������
	 
      
     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
	 EEPROM_WriteBytes(I2c_Buf_Write, 12, 1);  
     HAL_Delay(100);
	
     //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
   	 re=EEPROM_ReadBytes(I2c_Buf_Read, 12, 1); 
	 if(I2c_Buf_Read[0] != I2c_Buf_Write[0])
      {
        LED1_ON;
		LED2_ON;
		while(1); //wt.edit
      }
     
     
     }
    if(re==1) //��ʾд��ɹ�
        {
        LED1_ON;
		LED2_ON;
		HAL_Delay(50);
		LED1_OFF;
		LED2_OFF;
    	}
}	
	
uint8_t LAMP_Read_BrightValue(void)
{
       uint8_t flag;
	   uint8_t temp;
	   uint8_t sendbuffer[6]={0xa1,0x04,00,00,00,0x0b};
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		{
    	 I2c_Buf_Read[0]=0;
		//��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
       	 EEPROM_ReadBytes(I2c_Buf_Read, 12, 1); 
		 temp=I2c_Buf_Read[0];
		 }
		sendbuffer[4]=I2c_Buf_Read[0];
		printf("Lamp Brightness is %d \n",temp);
	    HAL_UART_Transmit(&husartx,sendbuffer,6,12);	
		return temp;

}
/**************************************************
*
*�������ƣ�
*�������ܣ����ذ��ȡ���ڶ����������ֵ
*������
*����ֵ:
*
**************************************************/
uint8_t A2_LAMP_Read_BrightValue(void)
{
      
	   uint8_t temp;
	   uint8_t sendbuffer[6]={0xa2,0x04,00,00,00,0x0b};
	   temp=I2C_RX_SAVE_Buffer[2];
	    sendbuffer[4]=temp;
	    printf("Lamp Brightness is %d \n",temp);
	    HAL_UART_Transmit(&husartx,sendbuffer,6,12);	
		return temp;

}

