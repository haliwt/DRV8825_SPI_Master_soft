#include "a1_fun/a1_fun.h"
#include "stm32f1xx_hal.h"
#include "usart/bsp_usartx.h"
#include "string.h"
#include "led/bsp_led.h"
#include "StepMotor/bsp_StepMotor.h"
#include "key/bsp_key.h"
#include "AdvancedTIM/bsp_AdvancedTIM.h"
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "drv8825/drv8825.h"
#include "i2c/bsp_EEPROM.h"
#include <math.h>
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "lamp/bsp_lamp.h"
#include "ds18b20/bsp_ds18b20.h"
#include "spi/bsp_spi.h"
#include "spi_comm/spi_comm.h"
#include "i2c_slave/bsp_I2C.h"

#define SENDBUFF_SIZE             100 // ����DMA���ͻ�������С

extern  __IO uint16_t  home_position    ; 
extern __IO uint8_t Brightness;   
extern __IO uint8_t save_flag;
extern __IO uint8_t re_intrrupt_flag; //����λ���������źű�־λ�������жϱ�־λ
extern __IO uint16_t judge_data;    //���յ����ݵĹ��������ݡ�
extern __IO uint8_t PB8_flag;
extern uint8_t SPI_TX_FLAG;
extern __IO uint16_t A1_Read_A2_Judge; //���1��ȡ���2 �ж�ֵָ��
extern __IO uint8_t  END_A1_READ_A2_RealTime_FLAG; //���1 ��ȡ���2 ֹͣ��־λ

/* ˽�б��� ------------------------------------------------------------------*/
extern uint8_t aRxBuffer[256];                      // �������� 
extern uint8_t aTxBuffer[SENDBUFF_SIZE];       // ����DMA���ͻ�����
extern uint8_t repcdata[3];                    //����λ�����յ���3���ֽ����ݡ�
extern uint8_t SPI_aRxBuffer[7];
extern uint8_t SPI_aTxBuffer[7];
extern uint8_t I2C_RX_Buffer[3];  //I2C ���յ�������
extern uint8_t I2C_RX_SAVE_Buffer[3];
extern __IO uint8_t A1_Read_FLAG;
/*************************************************************
 *
 *�������ƣ�
 *�������ܣ��������1��������ָ���
 *��������
 *����ֵ����
 *
**************************************************************/

void A1_FUN(void)
{
  
		
		if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
		{
		  DRV8825_StopMove();
		}
        switch(judge_data)
		  {
             
			  case 0x02 :   //??????????????????��?��???
				    re_intrrupt_flag=0;
					PB8_flag=0;
			  	   DRV8825_CW_AxisMoveRel(repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
				   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
 
			  break;
					
			  case 0x82 :   //????????????????
					  {
					  re_intrrupt_flag=0;
					  PB8_flag=0;
					  DRV8825_CCW_AxisMoveRel(repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
			          __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					  }
				    break;
			  case 0x33 :
					  {
						re_intrrupt_flag=0; 
						PB8_flag=0;
                       STEPMOTOR_PC_AxisMoveAbs( repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
                       __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					   }
			        
					  break;
					  
			  case 0xb0 :
			     {
			     re_intrrupt_flag=0;
				 PB8_flag=0;
			     //home_position = Read_Origin_Position();
				 STEPMOTOR_AxisMoveAbs(0*SPR,Toggle_Pulse);
				  __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
				 }
			  break;

			 case 0xa0 :    //???????????
			 	    re_intrrupt_flag=0; 
			 	    Set_NewOrigin_Position();
                    HAL_Delay(100);
                    __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					HAL_Delay(30);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(30);
					LED2_ON;
					LED1_ON;
					HAL_Delay(10);
			 	break;
			case 0x90 :
                     re_intrrupt_flag=0; 
			        DRV8825_SetSpeed(aRxBuffer[4],aRxBuffer[5]);
					HAL_Delay(30);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(30);
					LED2_ON;
					LED1_ON;
				break;
		   case 0xff :  //???��????
                    re_intrrupt_flag=0; 
					LED2_ON;
					LED1_ON;		  
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
		   	break;
			case 0xc0 :
                    re_intrrupt_flag=0; 
				   Brightness=aRxBuffer[5];
				   LAMP_Save_BrightValue(Brightness);
				   GENERAL_TIMx_Init();
				   HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4);
				
					LED2_ON;
					LED1_OFF;
					HAL_Delay(10);
				    LED2_OFF;
					LED1_ON;
				break;
			case 0xd0 :
                    re_intrrupt_flag=0; 
					EEPROM_Clear_Buf();
                    HAL_Delay(10);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(10);
					LED2_ON;
					LED1_ON;
					HAL_Delay(10);
				break;

           default:re_intrrupt_flag=0;
			   	     
				
		    }

		

}
/**********************************************************************
   *
   *�������ƣ�
   *�������ܣ����A1 ��ȡ���ݺ���
   *���뺯������
   *����ֵ����
   *
************************************************************************/
void A1_ReadData_FUN(void)
{
        uint8_t temp;//rxspidata;
        switch(judge_data)
          	{

			   case 0x103 :   //��ȡ���ʵʱλ��
					 A1_Read_FLAG=0;
					Display_EEPROM_Value();  //????????��???????
					__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					break;
				case 0x1e0:
					A1_Read_FLAG=0;
					A1_ReadEprom_Value();
				   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					break;
				case 0x104 :
					    A1_Read_FLAG=0;
					    temp= LAMP_Read_BrightValue(); //????????
						printf("BRV = %d \n",temp);
						__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					
					break;
				case 0x101:
	                A1_Read_FLAG=0; 
					PC_DRV8825_ReadSpeed(); //??????????
					__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					break;
				default:A1_Read_FLAG=0;
          	}

}


/**********************************************************************
   *
   *�������ƣ�
   *�������ܣ����Ƶڶ�����ﺯ��
   *���뺯������
   *����ֵ����
   *
************************************************************************/
void A1_CONTROL_A2_FUN(void)
{
        uint8_t spi_order;
        if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
		{
		     DRV8825_StopMove();
		}
		 switch(judge_data)
           	{
			case 0x202 :   //??????????????????��?��???
				     END_A1_READ_A2_RealTime_FLAG=0;
			         SPI_aTxBuffer[1]=0x00;
			         spi_order=0x02;
					 SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			
 
			  break;
					
			  case 0x282 :   //????????????????
					  {
						 END_A1_READ_A2_RealTime_FLAG=0;
						 SPI_aTxBuffer[1]=0x00;
						 spi_order=0x82;
						 SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
						  __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					  }
				    break;
			  case 0x233 :
					  {
						END_A1_READ_A2_RealTime_FLAG=0;
                        SPI_aTxBuffer[1]=0x00;						  
						spi_order=0x33;
						SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
						 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					  }
			        
					  break;
					  
			  case 0x2b0 :
			       {
			      
                   END_A1_READ_A2_RealTime_FLAG=0;
				   SPI_aTxBuffer[1]=0x00;
                   SPI_aTxBuffer[1]=0x00;					   
                   spi_order=0xb0;
				   SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
					__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			        }
				   
			  
			 break;

			 case 0x2a0 :    //???????????
			 	    END_A1_READ_A2_RealTime_FLAG=0;
			        SPI_aTxBuffer[1]=0x00;
                    spi_order=0xa0;
				    SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
                     __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			 	break;
			case 0x290 :     //
                     END_A1_READ_A2_RealTime_FLAG=0;
			         SPI_aTxBuffer[1]=0x00;
			         spi_order=0x90;
			         SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				break;
		   case 0x2ff :  //???��????
                     END_A1_READ_A2_RealTime_FLAG=0;
		             SPI_aTxBuffer[1]=0x00;
		             spi_order=0xff;
				     SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
		             __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				  break;
			case 0x2c0 :  //LED ???????,?????
                     END_A1_READ_A2_RealTime_FLAG=0;
                     SPI_aTxBuffer[1]=0x00;			
			         spi_order=0xc0;
				     SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			          __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
				break;
			case 0x2d0 :
                    END_A1_READ_A2_RealTime_FLAG=0;
			        SPI_aTxBuffer[1]=0x00;
					 spi_order=0xd0;
				     SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
				break;
			case 0x00 :
					 
			         SPI_aTxBuffer[1]=0x00;
                     spi_order=00;						 
					 SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			        __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					  break;
			case 0x2ee:    //A1 ?? A2 ???????
			
			    SPI_TX_FLAG=0;
			    SPI_aTxBuffer[1]=0x00;
			    spi_order=0xee;
			    SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			        LED2_ON;
					LED1_ON;		  
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				break;
		 	}
}

/***********************************************
   *
   *�������ƣ�
   *�������ܣ����1��ȡ�ڶ��������
   *�����������
   *����ֵ����
   *
************************************************/
void A1_ReadData_A2_Fun(void)	
{
           uint8_t spi_order,temp;
           switch(A1_Read_A2_Judge)
           {
            
			case 0x2103 :   //?????? ?????��??????? 0x 1xx A2
				  SPI_aTxBuffer[1]=0x01;
			      spi_order=0x03;
			      SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			      HAL_Delay(50);
				  if(END_A1_READ_A2_RealTime_FLAG==1)
				  {
                    printf("A1 read A2 real time over \n");
			      }
			      else
			      A2_Pulse_RealTime_Value();
			       __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				break;
			
			case 0x2104 :   //???LED???????
				   
			        SPI_aTxBuffer[1]=0x01;
			        spi_order=0x04;
			        SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         HAL_Delay(100);
			         temp= A2_LAMP_Read_BrightValue();
			         printf("A2_BRV = %d \n",temp);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				break;
			case 0x2102 :
				   
			       	SPI_aTxBuffer[1]=0x01;
			        spi_order=0x02;
			        SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
					__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
				break;
			case 0x2101: // Read the second motor of speed value A2.
				
				SPI_aTxBuffer[1]=0x01;
			    spi_order=0x01;
			    SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			    HAL_Delay(10);
			    PC_A2_DRV8825_ReadSpeed(I2C_RX_SAVE_Buffer[1],I2C_RX_SAVE_Buffer[2]); //??????????
			   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			    break;
			case 0x21e0:  //????????????????
			
			     SPI_aTxBuffer[1]=0x01;
			     spi_order=0xe0;
			     SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			      HAL_Delay(200);
			      PC_A2_Pulse_EEPROM_Value();
				__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			break;
			default :
				break;
		 }

}












