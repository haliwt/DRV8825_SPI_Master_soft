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

extern uint8_t aRxBuffer[48];                      // �������� 
extern uint8_t aTxBuffer[SENDBUFF_SIZE];       // ����DMA���ͻ�����

/********************************************
*ͬ��λ��ͨѶ����
*���ܣ���λ��������λ��ָ��
*
******************************************/

/********************************************************************************************************************************
 *
 *�������ܣ�PC����λ����д����
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
		  case 0x00 : //���˳ʱ����ת
			{ 
								
			}
			break;
			case 0x01 : //�����ʱ�뷽����ת
			{ 
								
			}
			break;
			
			case 0x0f :   //����ٶ�����
								{
									recedat3=aRxBuffer[3];
									recedat4=aRxBuffer[4];
									if(aRxBuffer[5]==0x0b)
									{
										DRV8825_SetSpeed(recedat3,recedat4);
									}
								}
																	
								case 0x10 :   //���ǰ���ƶ�
								if(aRxBuffer[5]==0x0b)
								{
                  DRV8825_CW_NormalMove();
								  
                }
                                
								break;
								
								case 0x20 :   //������
								if(aRxBuffer[5]==0x0b)
								{
                  DRV8825_CCW_NormalMove();
								
                }
								break;
								
								case 0x11 :  //������λ���ƶ� ˳ʱ��
								  recedat3=aRxBuffer[3];
									recedat4=aRxBuffer[4];
								  mult=recedat3<<8|recedat4;
									if(aRxBuffer[5]==0x0b)
									{
										DRV8825_CW_RelativeDisantce(mult);
									}
							
								break;

							case 0x21:  //������λ���ƶ�  ��ʱ��ת��
								recedat3=aRxBuffer[3];
								recedat4=aRxBuffer[4];
								 mult2=recedat3<<8|recedat4;
							  if(aRxBuffer[5]==0x0b)
								{
									DRV8825_CCW_RelativeDisantce(mult2);
							 
								}
								// rscounter = 0;
							 break;
                                    
              /*****************���ԭ������**************************/	
							case 0x08:  //���ԭ������
							{
									  recedat3=aRxBuffer[3];
                    recedat4=aRxBuffer[4];
								    if(aRxBuffer[5]==0x0b)
								    {
									  
                     DRV8825_SetOriginPos(recedat3,recedat4);
								   }
							 }
							break;
								
							
								/*******************������λ������********************/
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
								case  0x50:     //���ֹͣ�ƶ�
								if(aRxBuffer[5]==0x0b)
								{
                  DRV8825_Stop_Move();
									
                }
								break;
								
								
               /*****************�ƹ�����**************************/								
								case 0x70 :    
								{
								  recedat3=aRxBuffer[3];
                  recedat4=aRxBuffer[4];
								  //mult3=recedat3<<8|recedat4;
                  if(aRxBuffer[5]==0x0b)
									{
									
										printf ( "\r\n ���Ȧ���� %d\r\n", recedat);		
                    										
									}
									
								}	         
								break; 
							
							/*******************************************/
								
								case 0x80 :    //��λ����������λ����ִ�в��� OK ��Ϣ
									printf ( "\r\n works ok %d\r\n", 0X80);		
								break; 
							
            /*********************���ʹ������**********************/	
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
			////////////��ȡָ��/////////////////////
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
								
                
								//rscounter = 0;  //�����rscounter ��ʾ�������¶�ֵ������
							}
							 
							break;
						
						
						}
						}
				} //end if(aRxBuffer[0]==0xa1)
						 
		
}  //end fuction
			
	
	
			
		
			
			
			
   
		

	




				
				
				
						
						
									
										
								
							
					
				
			
									  
			


