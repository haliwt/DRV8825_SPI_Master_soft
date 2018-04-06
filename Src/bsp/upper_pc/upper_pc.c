#include "upper_pc.h"
#include <stdio.h>

#include "led/bsp_led.h"
#include "drv8825/drv8825.h"
#include "StepMotor/bsp_STEPMOTOR.h" 
#include "usart/bsp_usartx.h"
//#include "bsp_ds18b20.h"
//#include "usart.h"
//#include "lamp_pwm_output.h"
//#include "bsp_adc.h"
//#include "usart.h"
#define SENDBUFF_SIZE 100

extern uint8_t aRxBuffer[48];                      // 接收数据 
extern uint8_t aTxBuffer[SENDBUFF_SIZE];       // 串口DMA发送缓冲区

/********************************************
*同上位机通讯函数
*功能：上位机操作下位机指令
*
******************************************/

/********************************************************************************************************************************
 *
 *函数功能：PC对下位机读写函数
 *
 *
 **********************************************************************************************************************************/
void PC_ReadWrite_Fun(void)
{
  
  uint16_t mult,mult2;
  uint8_t recedat,recedat1,recedat3,recedat4;
	if(aRxBuffer[0]==0xa1)
	{
	  recedat1=aRxBuffer[2];
		switch(recedat1)
		{
		  case 0x00 : //马达顺时针旋转
			{ 
								
			}
			break;
			case 0x01 : //马达逆时针方向旋转
			{ 
								
			}
			break;
			
			case 0x0f :   //马达速度设置
								{
									recedat3=aRxBuffer[3];
									recedat4=aRxBuffer[4];
									if(aRxBuffer[5]==0x0b)
									{
										DRV8825_SetSpeed(recedat3,recedat4);
									}
								}
																	
								case 0x10 :   //马达前进移动
								if(aRxBuffer[5]==0x0b)
								{
                  DRV8825_CW_NormalMove();
								  
                }
                                
								break;
								
								case 0x20 :   //马达后退
								if(aRxBuffer[5]==0x0b)
								{
                  DRV8825_CCW_NormalMove();
								
                }
								break;
								
								case 0x11 :  //马达相对位置移动 顺时针
								  recedat3=aRxBuffer[3];
									recedat4=aRxBuffer[4];
								  mult=recedat3<<8|recedat4;
									if(aRxBuffer[5]==0x0b)
									{
										DRV8825_CW_RelativeDisantce(mult);
									}
							
								break;

							case 0x21:  //马达相对位置移动  逆时针转动
								recedat3=aRxBuffer[3];
								recedat4=aRxBuffer[4];
								 mult2=recedat3<<8|recedat4;
							  if(aRxBuffer[5]==0x0b)
								{
									DRV8825_CCW_RelativeDisantce(mult2);
							 
								}
								// rscounter = 0;
							 break;
                                    
              /*****************马达原点设置**************************/	
							case 0x08:  //马达原点设置
							{
									  recedat3=aRxBuffer[3];
                    recedat4=aRxBuffer[4];
								    if(aRxBuffer[5]==0x0b)
								    {
									  
                     DRV8825_SetOriginPos(recedat3,recedat4);
								   }
							 }
							break;
								
							
								/*******************马达绝对位置设置********************/
								case 0x33 :  //
								{
									 recedat3=aRxBuffer[3];
                    recedat4=aRxBuffer[4];
								
								    if(aRxBuffer[5]==0x0b)
								     {
									      DRV8825_CCCW_AbsPosMove(recedat3,recedat4);
											 
										 }
								     
								}
								break;
						    
								
								/**************************************/
								case  0x50:     //马达停止移动
								if(aRxBuffer[5]==0x0b)
								{
                  DRV8825_Stop_Move();
									
                }
								break;
								
								
               /*****************灯光设置**************************/								
								case 0x70 :    
								{
								  recedat3=aRxBuffer[3];
                  recedat4=aRxBuffer[4];
								  //mult3=recedat3<<8|recedat4;
                  if(aRxBuffer[5]==0x0b)
									{
									
										printf ( "\r\n 马达圈数： %d\r\n", recedat);		
                    										
									}
									
								}	         
								break; 
							
							/*******************************************/
								
								case 0x80 :    //下位机反馈给上位机的执行操作 OK 信息
									printf ( "\r\n works ok %d\r\n", 0X80);		
								break; 
							
            /*********************马达使能命令**********************/	
								case 0xb0 :
								{
									if(aRxBuffer[5]==0x0b)
									{
									DRV8825_Turn_ENABLE();
								 
									LED1_ON;
								  LED2_OFF;
									}
								}
								
                break;	
									
				} //end 	if(aRxBuffer[1]==0x10)
			////////////读取指令/////////////////////
				if(aRxBuffer[1]==0x11)
				{
				   recedat1=aRxBuffer[2];
					  switch(recedat1)
						{
						  case 0x90:
							{
							  if(aRxBuffer[5]==0x0b)
								{
								  
		   
	               }	 
								
                
								//rscounter = 0;  //清楚了rscounter 显示不出来温度值，乱码
							}
							 
							break;
						
						
						}
						}
				} //end if(aRxBuffer[0]==0xa1)
						 
		
}  //end fuction
			
	
	
			
		
			
			
			
   
		

	




				
				
				
						
						
									
										
								
							
					
				
			
									  
			


