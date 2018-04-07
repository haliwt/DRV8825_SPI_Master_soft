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

#define SENDBUFF_SIZE             100 // 串口DMA发送缓冲区大小

extern  __IO uint16_t  home_position    ; 
extern __IO uint8_t Brightness;   
extern __IO uint8_t save_flag;
extern __IO uint8_t re_intrrupt_flag; //从上位机，接收信号标志位，进入中断标志位
extern __IO uint16_t judge_data;    //接收到数据的功能码数据。
extern __IO uint8_t PB8_flag;
extern uint8_t SPI_TX_FLAG;
/* 私有变量 ------------------------------------------------------------------*/
extern uint8_t aRxBuffer[256];                      // 接收数据 
extern uint8_t aTxBuffer[SENDBUFF_SIZE];       // 串口DMA发送缓冲区
extern uint8_t repcdata[3];                    //从上位机接收到的3个字节数据。
extern uint8_t SPI_aRxBuffer[7];
extern uint8_t SPI_aTxBuffer[7];
extern uint8_t I2C_RX_Buffer[3];  //I2C 接收到的数据
extern uint8_t I2C_RX_SAVE_Buffer[3];
extern __IO uint8_t A2_FLAG;

/*************************************************************
 *
 *函数名称：
 *函数功能：操作马达1，和马达的指令函数
 *参数：无
 *返回值：无
 *
**************************************************************/

void A1_FUN(void)
{
  
		uint8_t DS18B20ID[8],temp;//rxspidata;
        float temperature;
		if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
		{
		  DRV8825_StopMove();
		}
        switch(judge_data)
		  {
              if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)//||(stop_key_flag==1))
		      {
		       PB8_flag=1;
			   DRV8825_StopMove();
		      }
			  case 0x02 :   //??????????????????λ?÷???
				    re_intrrupt_flag=0;
					PB8_flag=0;
			  	   // DRV8825_SLEEP_DISABLE(); //???????????,?????????
			  	   // HAL_Delay(10);
					DRV8825_CW_AxisMoveRel(repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
					if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					{
						DRV8825_StopMove();
					}
					HAL_Delay(2);
					if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					{
						DRV8825_StopMove();
					}
					LED2_OFF;
					LED1_ON;
					HAL_Delay(2);
					if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					{
						DRV8825_StopMove();
					}
				
					if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					{
						DRV8825_StopMove();
					}
					 repcdata[0]=0;
			          repcdata[1]=0;
					  repcdata[2]=0;
 
			  break;
					
			  case 0x82 :   //????????????????
					  {
					  re_intrrupt_flag=0;
					  PB8_flag=0;
					 // DRV8825_SLEEP_DISABLE(); //???????????,?????????	
					 //  HAL_Delay(10);
			          DRV8825_CCW_AxisMoveRel(repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
			          if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}					
					HAL_Delay(2);
					LED2_ON;
					LED1_OFF;
					 if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}	
					HAL_Delay(2);
					//STEPMOTOR_AxisMoveRel(335544*SPR, Toggle_Pulse);---?????????
				
					//__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					 if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}
					  repcdata[0]=0;
			          repcdata[1]=0;
					  repcdata[2]=0;
                       // HAL_UART_Transmit(&husartx,tranbuffer,1,1);
					  }
				    break;
			  case 0x33 :
					  {
						re_intrrupt_flag=0; 
						PB8_flag=0;
                       if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					   {
							DRV8825_StopMove();
					   }	
					  STEPMOTOR_PC_AxisMoveAbs( repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
                      
					   if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}	
						
					 // printf("0X33 is OK \n");
					  repcdata[0]=0;
			          repcdata[1]=0;
					  repcdata[2]=0;
					
					  }
			        
					  break;
					  
			  case 0xb0 :
			     {
			     re_intrrupt_flag=0;
				 PB8_flag=0;
				 // DRV8825_SLEEP_DISABLE() ; //???????????
				 // HAL_Delay(10);
				  home_position = Read_Origin_Position();
				 STEPMOTOR_AxisMoveAbs(0*SPR,Toggle_Pulse);
				 if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
				 {
					DRV8825_StopMove();
				 }
				 }
			  break;

			 case 0xa0 :    //???????????
			 	    re_intrrupt_flag=0; 
			 	    Set_NewOrigin_Position();
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
		   case 0xff :  //???λ????
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

            case 0x103 :   //??????  0x 1xx 
				 re_intrrupt_flag=0; 
				Display_EEPROM_Value();  //????????λ???????
				break;
			case 0x104 :
				    re_intrrupt_flag=0; 
				    temp= LAMP_Read_BrightValue(); //????????
					printf("BRV = %d \n",temp);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(20);
					LED2_ON;
					LED1_ON;
					HAL_Delay(20);
				break;
			case 0x102 :
				    re_intrrupt_flag=0; 
				     temperature=DS18B20_GetTemp_MatchRom(DS18B20ID);
					/* ?????? DS18B20 ???к????????? */
					printf("????????к???????????%.1f\n",temperature);
					/* 1s ?????????? */
					HAL_Delay(1000);
					printf("????????к???????????%.1f\n",temperature);
					HAL_Delay(1000);
					
				break;
			case 0x101:
                re_intrrupt_flag=0; 
				PC_DRV8825_ReadSpeed(); //??????????
				break;
	
				
			break;
			default:re_intrrupt_flag=0;
			   	     
				
		    }

		

}


/**********************************************************************
   *
   *函数名称：
   *函数功能：控制第二个马达函数
   *输入函数：无
   *返回值：无
   *
************************************************************************/
void A1_CONTROL_A2_FUN(void)
{
        uint8_t spi_order,temp;
        if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
		{
		     DRV8825_StopMove();
		}
		 switch(judge_data)
           	{
			case 0x202 :   //??????????????????λ?÷???
				     A2_FLAG=0;
			         SPI_aTxBuffer[1]=0x00;
			         spi_order=0x02;
					 SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			
 
			  break;
					
			  case 0x282 :   //????????????????
					  {
						 A2_FLAG=0;
						 SPI_aTxBuffer[1]=0x00;
						 spi_order=0x82;
						 SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
						  __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					  }
				    break;
			  case 0x233 :
					  {
						A2_FLAG=0;
                        SPI_aTxBuffer[1]=0x00;						  
						spi_order=0x33;
						SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
						 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					  }
			        
					  break;
					  
			  case 0x2b0 :
			       {
			       A2_FLAG=0;
				   SPI_aTxBuffer[1]=0x00;
                   SPI_aTxBuffer[1]=0x00;					   
                   spi_order=0xb0;
				   SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
					__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			        }
				   
			  
			 break;

			 case 0x2a0 :    //???????????
			 	    A2_FLAG=0;
			        SPI_aTxBuffer[1]=0x00;
                    spi_order=0xa0;
				    SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			 	break;
			case 0x290 :     //
                     A2_FLAG=0; 
			         SPI_aTxBuffer[1]=0x00;
			         spi_order=0x90;
			         SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				break;
		   case 0x2ff :  //???λ????
                     A2_FLAG=0;
		             SPI_aTxBuffer[1]=0x00;
		             spi_order=0xff;
				     SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
		             __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				  break;
			case 0x2c0 :  //LED ???????,?????
                     A2_FLAG=0; 
                     SPI_aTxBuffer[1]=0x00;			
			         spi_order=0xc0;
				     SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			          __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
				break;
			case 0x2d0 :
                    A2_FLAG=0; 
			        SPI_aTxBuffer[1]=0x00;
					 spi_order=0xd0;
				     SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
				break;
			case 0x00 :
					A2_FLAG=0; 
			         SPI_aTxBuffer[1]=0x00;
                     spi_order=00;						 
					 SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					  break;
			case 0x2ee:    //A1 ?? A2 ???????
				A2_FLAG=0;
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
		   /*********************************************************************************/
          /***************************控制第二个马达***************************************/
            case 0x2103 :   //?????? ?????λ??????? 0x 1xx A2
				  A2_FLAG=0;
                  SPI_aTxBuffer[1]=0x01;
			      spi_order=0x03;
			      SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			      HAL_Delay(100);
			      A2_Pulse_RealTime_Value();
			       __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				break;
			case 0x2104 :   //???LED???????
				    A2_FLAG=0;
			        SPI_aTxBuffer[1]=0x01;
			        spi_order=0x04;
			        SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			         HAL_Delay(100);
			         temp= A2_LAMP_Read_BrightValue();
			         printf("A2_BRV = %d \n",temp);
			         __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
				break;
			case 0x2102 :
				    A2_FLAG=0;
			       	SPI_aTxBuffer[1]=0x01;
			        spi_order=0x02;
			        SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
					 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
				break;
			case 0x2101: // Read the second motor of speed value A2.
				A2_FLAG=0; 
				SPI_aTxBuffer[1]=0x01;
			    spi_order=0x01;
			    SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			    HAL_Delay(10);
			    PC_A2_DRV8825_ReadSpeed(I2C_RX_SAVE_Buffer[1],I2C_RX_SAVE_Buffer[2]); //??????????
			    __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			    break;
			case 0x21e0:  //????????????????
				 A2_FLAG=0; 
			     SPI_aTxBuffer[1]=0x01;
			     spi_order=0xe0;
			     SPI_COMM_Function(spi_order,repcdata[0],repcdata[1],repcdata[2]);
			      HAL_Delay(200);
			      PC_A2_Pulse_EEPROM_Value();
				 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
			break;
			default :A2_FLAG=0;
		 }

}












