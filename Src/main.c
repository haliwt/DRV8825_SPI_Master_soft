
/* ����ͷ�ļ� ----------------------------------------------------------------*/
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

#include "a1_fun/a1_fun.h"

#define SENDBUFF_SIZE             100 // ����DMA���ͻ�������С
#define STEPMOTOR_MICRO_STEP      32  // �������������ϸ�֣�������������ʵ�����ö�Ӧ

/* ˽�б��� ------------------------------------------------------------------*/
uint8_t aRxBuffer[256];                      // �������� 
uint8_t aTxBuffer[SENDBUFF_SIZE];       // ����DMA���ͻ�����
uint8_t repcdata[3]={0x00,0x00,0x00};                    //����λ�����յ���3���ֽ����ݡ�
extern uint8_t SPI_aRxBuffer[7];
extern uint8_t SPI_aTxBuffer[7];
uint8_t I2C_RX_Buffer[3];  //I2C ���յ�������
uint8_t I2C_RX_SAVE_Buffer[3];
/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint16_t Toggle_Pulse; /* ��������ٶȿ��ƣ��ɵ��ڷ�ΧΪ 650 -- 3500 ��ֵԽС�ٶ�Խ�� */
__IO uint32_t pulse_count; /*  ���������һ�����������������2 */
extern  __IO uint16_t  home_position    ; 
extern __IO uint8_t Brightness;   
extern __IO uint8_t save_flag;
__IO uint8_t re_intrrupt_flag=0; //����λ���������źű�־λ�������жϱ�־λ
__IO uint16_t judge_data;    //���յ����ݵĹ��������ݡ�
extern __IO uint8_t key_stop;
__IO uint8_t PB8_flag=0;
extern __IO uint8_t stop_key_flag;
uint8_t SPI_TX_FLAG=0;
extern uint8_t test_aTxBuffer[7];
uint8_t i2c_rx_data;
static uint8_t j=0;
extern __IO uint8_t END_STOP_FLAG;  //������е��յ㣬ֹͣ��־�
__IO uint8_t A2_FLAG=0;
__IO uint8_t A1_Read_FLAG=0; //A1 读取参数值

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9��Ƶ���õ�72MHz��ʱ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ�72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1ʱ�ӣ�36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2ʱ�ӣ�72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // ���ò�����ϵͳ�δ�ʱ��
 // HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100000);  // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/*********************************************************
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��   SPI1 ͨ�Ŵӻ�
  *********************************************************/
int main(void)
{
  uint8_t txbuf[100],Mode_Count=0;
  // uint8_t tranbuffer[]={0x88};
   uint8_t DS18B20ID[8],i;//rxspidata;
   
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  LED_GPIO_Init();
  KEY_GPIO_Init();
  GENERAL_TIMx_Init();
	
  MX_USARTx_Init();
  STEPMOTOR_TIMx_Init();
  MX_I2C_EEPROM_Init();
  /* ȷ����ʱ�� */
  SPIx_Init();
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);

  // __HAL_UART_ENABLE_IT(&husartx, UART_IT_IDLE);  //wt.edit 11.07
  memcpy(txbuf,"SPI_MASTER version6.01 2018-04-07 \n",100);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  Brightness=LAMP_Read_BrightValue(); 
  GENERAL_TIMx_Init();
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4); 
  /* ʹ�ܽ��գ������жϻص����� */
  HAL_UART_Receive_IT(&husartx,aRxBuffer,7);
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_1);//HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_1);
   /* ʹ�ܽ��գ������жϻص����� */
  //Brightness=LAMP_Read_BrightValue(); 
  GENERAL_TIMx_Init();
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4);  
   printf("DS18B20�¶ȴ�������Ϣ��ȡ\n");
  while(DS18B20_Init())	
  {
		printf("DS18B20�¶ȴ�����������\n");    
        HAL_Delay(1000);
	    break;
  }
  printf("��⵽DS18B20�¶ȴ�����������ʼ���ɹ�\n");
  DS18B20_ReadId(DS18B20ID);
  printf("DS18B20�����к��ǣ� 0x");  
	for ( i = 0; i < 8; i ++ )             
	  printf ( "%.2X", DS18B20ID[i]);
  printf("\n");  
  
  DS18B20_GetTemp_MatchRom(DS18B20ID);
  HAL_Delay(100);
  
  SPIx_CS_ENABLE() ;
  HAL_Delay(600);
  MX_I2C_EEPROM_Init();
  
  HAL_I2C_Slave_Receive_IT(&I2cHandle,&i2c_rx_data,4);

  
  while (1)
  {
		
	     DRV8825_SLEEP_DISABLE() ; //�ߵ�ƽ��ʼ����
         if(KEY1_StateRead()==KEY_DOWN)
		 {
		     Mode_Count++;
			   if(Mode_Count ==4&&Mode_Count!=0)
               Mode_Count = 0;	 
			   STEPMOTOR_AxisMoveRel(-5*SPR, Toggle_Pulse);
			  
		}
		if(KEY2_StateRead() == KEY_DOWN)
		{
			switch(Mode_Count)
			{
				case 0:
				{
				 	//STEPMOTOR_AxisHome(0x03); //����ԭ��
				}break;
				case 1://CW˳ʱ�뷽����תn*SPR��ʾ��ת3Ȧ
				{
					STEPMOTOR_AxisMoveRel(1*SPR, Toggle_Pulse);
				}break;
				case 2://CCW��ʱ�뷽����ת
				{
						STEPMOTOR_AxisMoveRel(-1*SPR, Toggle_Pulse);
				}break;
				case 3://������ԭ�㹦��,�Զ��ص�����
				{
					STEPMOTOR_AxisMoveAbs(0*SPR,Toggle_Pulse);
				}
				default :break;			
			}
		}
		
		if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)//||(stop_key_flag==1))
		{
		    PB8_flag=1;
			DRV8825_StopMove();
			//printf("key stopmove is ok\n");
		 }
		if(re_intrrupt_flag==1) 
		{
           A1_FUN();
		}
		if(A1_Read_FLAG==1)
		{
            A1_Read_FLAG=0;
			A1_ReadData_FUN();
		}
		if(A2_FLAG==1)
		{
            A1_CONTROL_A2_FUN();
		}
		if( END_STOP_FLAG==1)  //���е��յ㣬ֹͣ��־λ��
		{
		   END_STOP_FLAG=0;
		   Motor_Save_EndPosition();
        }
		
	}
	
}
/*****************************end main()***************************************************/
/***********************************************************************************/
/**************************************************************************************/






/*******************************************************************************
  * ��������: ���ڽ�����ɻص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  ******************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
   
    __HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
	if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
		{
		  DRV8825_StopMove();
		}
  
if(HAL_UART_Receive_IT(&husartx,aRxBuffer,7)==HAL_OK )
 {
	  //HAL_Delay(50);
		if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
		{
		  DRV8825_StopMove();
		}

	  if(aRxBuffer[0]==0xa1)
		{
          if(aRxBuffer[1]==0x00)
		   {
			switch(aRxBuffer[2])
            	{
                      case 0xff :
                         if(aRxBuffer[6]==0x0b)
                         	{
                               re_intrrupt_flag=1;
				               judge_data= 0xff;
					
					          __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						     }
					  
					  	break;
					  case 0x33 :
						  if(aRxBuffer[6]==0x0b)
                         	{
                                re_intrrupt_flag=1;
								repcdata[0]=	aRxBuffer[3]; //����ֽ�
							    repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
							    repcdata[2]=	aRxBuffer[5]; //����ֽ�
							    judge_data= 0x33;
							    __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	  
						     
							}
						  
						  break;
						case 0xb0 :
						  if(aRxBuffer[6]==0x0b)
                         	{
                               re_intrrupt_flag=1;
				              judge_data= 0xb0;
				
				               __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
						    
							}
						  
						  break;
						case 0x02 :
							 if(aRxBuffer[6]==0x0b)
								{
									re_intrrupt_flag=1;
								   repcdata[0]=	aRxBuffer[3]; //����ֽ�
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x02;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 
								}
							
							break;
						case 0x82 :
							if(aRxBuffer[6]==0x0b)
								{
									re_intrrupt_flag=1;
									repcdata[0]=	aRxBuffer[3]; //����ֽ�
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x82;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
								}
							
							break;
						case 0x90 :
							if(aRxBuffer[6]==0x0b)
								{	  
									re_intrrupt_flag=1;
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x90;
									
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							
							break;
						case 0xa0 :
							if(aRxBuffer[6]==0x0b)   //��������ԭ��
								{
									//DRV8825_SLEEP_DISABLE(); //�ߵ�ƽ��ʼ����,�������״̬
									 re_intrrupt_flag=1;
									 judge_data= 0xa0;
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
									
								}
							break;
						case 0x00 :
							if(aRxBuffer[6]==0x0b)
								{
									DRV8825_StopMove();
									LED2_OFF;
									LED1_OFF;
									HAL_Delay(10);
									LED1_ON;
									LED2_ON;
							
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							break;
						case 0xc0 :
							 if(aRxBuffer[6]==0x0b)
								{
								   re_intrrupt_flag=1;
								   judge_data= 0xc0;
                                   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							break;
						case 0xd0 :
							 if(aRxBuffer[6]==0x0b)
								{
								   re_intrrupt_flag=1;
								   judge_data= 0xd0;
									 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
								}
							  break;
			    }
			} //end if(aRxBuffer[1]==0x00)
		/*************************A1读取马达参数值********************************/
		   if(aRxBuffer[1]==0x01)   
		   {
		     switch(aRxBuffer[2])
			 {
				 case 0x03:   //��ȡ���ʵʱλ��������
				    if(aRxBuffer[6]==0x0b)
						{
						   A1_Read_FLAG=1;
						   judge_data= 0x103;
						  //Display_CurrentPosition();
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
				  break; 
				case 0xe0:   //读取A1 EEPROM 值
				    if(aRxBuffer[6]==0x0b)
						{
						   A1_Read_FLAG=1;
						   judge_data= 0x1e0;
						   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
				  break; 
				 case 0x04:      /* ��ȡLED ������ֵ */
					 {
						if(aRxBuffer[6]==0x0b)
						{

							A1_Read_FLAG=1;
						   judge_data= 0x104;
							
						
						__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
				    break;
				  case 0x01 ://case 0x01 :     /*if(aRxBuffer[2]==0x01)��ȡ���ת��ֵ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{

							A1_Read_FLAG=1;
						   judge_data= 0x101;
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
					 break;
					 case 0x02:    //if(aRxBuffer[2]==0x02)  /*��ȡ�����¶�ֵ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{
							
						   A1_Read_FLAG=1;
						   judge_data= 0x102;
						  
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
                     break;
                   					 
				   
		   }
	     
	    }
  }// end if(aRxBuffer[0]==0xa1)

  /*****************控制第二个马达指令******************************/
   if(aRxBuffer[0]==0xA2)
   {
    if(aRxBuffer[1]==0x00)
		   {
			switch(aRxBuffer[2])
            	{
                      case 0xff :
                         if(aRxBuffer[6]==0x0b)
                         	{
                               A2_FLAG=1;
				               judge_data= 0x2ff;
					
					          __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						     }
					  
					  	break;
					  case 0x33 :
						  if(aRxBuffer[6]==0x0b)
                         	{
                                A2_FLAG=1;
								repcdata[0]=	aRxBuffer[3]; //����ֽ�
							    repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
							    repcdata[2]=	aRxBuffer[5]; //����ֽ�
							    judge_data= 0x233;
							    __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	  
						     
							}
						  
						  break;
						case 0xb0 :
						  if(aRxBuffer[6]==0x0b)
                         	{
                               A2_FLAG=1;
				              judge_data= 0x2b0;
				
				               __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
						    
							}
						  
						  break;
						case 0x02 :
							 if(aRxBuffer[6]==0x0b)
								{
									A2_FLAG=1;
								   repcdata[0]=	aRxBuffer[3]; //����ֽ�
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x202;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 
								}
							
							break;
						case 0x82 :
							if(aRxBuffer[6]==0x0b)
								{
									A2_FLAG=1;
								   repcdata[0]=aRxBuffer[3]; //����ֽ�
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x282;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
								}
							
							break;
						case 0x90 :
							if(aRxBuffer[6]==0x0b)
								{	  
									A2_FLAG=1;
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x290;
									
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							
							break;
						case 0xa0 :
							if(aRxBuffer[6]==0x0b)   //��������ԭ��
								{
									//DRV8825_SLEEP_DISABLE(); //�ߵ�ƽ��ʼ����,�������״̬
									 A2_FLAG=1;
									 judge_data= 0x2a0;
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
									
								}
							break;
						case 0x00 :   //���Ƶڶ���ֹͣ��
							if(aRxBuffer[6]==0x0b)
								{
									A2_FLAG=1;
									judge_data= 0x00;
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							break;
						case 0xc0 :
							 if(aRxBuffer[6]==0x0b)
								{
								   A2_FLAG=1;
								   judge_data= 0x2c0;
                                   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							break;
						case 0xd0 :  //���EEPROM����
							 if(aRxBuffer[6]==0x0b)
								{
								   A2_FLAG=1;
								   judge_data= 0x2d0;
									 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
								}
							  break;
					case 0xee:   //����A1 ��A2 ��ͨѶͬ����A1,A2 LEDͬʱ��˸
							 A2_FLAG=1;
							  judge_data= 0x2ee;
						       __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
							  break;
						
			    }
			} //end if(aRxBuffer[1]==0x00)
		   /*****************************��ȡ�ڶ����ָ��**********************************/
		   if(aRxBuffer[1]==0x01)   
		   {
		     switch(aRxBuffer[2])
			 {
				 case 0x03:   //��ȡ���ʵʱλ��������
				    if(aRxBuffer[6]==0x0b)
						{
						   A2_FLAG=1;
						   judge_data= 0x2103;
						  //Display_CurrentPosition();
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
				  break; 
				 case 0x04:      /* ��ȡLED ������ֵ */
					 {
						if(aRxBuffer[6]==0x0b)
						{

							A2_FLAG=1;
						   judge_data= 0x2104;
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
				    break;
				  case 0x01 ://case 0x01 :     /*if(aRxBuffer[2]==0x01)��ȡ���ת��ֵ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{
                          A2_FLAG=1;
						   judge_data= 0x2101;
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
					 break;
					 case 0x02:    //if(aRxBuffer[2]==0x02)  /*��ȡ�����¶�ֵ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{
							
						   A2_FLAG=1;
						   judge_data= 0x2102;
						  
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
                     break;	
					 case 0xe0 :
						 if(aRxBuffer[6]==0x0b)
						{
							
						   A2_FLAG=1;
						   judge_data= 0x21e0;
						  
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
						 break;
                   					 
				   
		   }
	     
	    }
    }  //end if(aRxBuffer[0]==0xa2)

}

}
/****************************************************************
*
*�������ܣ�SPI�жϣ��ص�����
*��������
*����ֵ����
*
*****************************************************************/
#if 1
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  
   SPI_TX_FLAG=1;
    //__HAL_SPI_DISABLE(&hspi_SPI);

  //HAL_SPI_Receive_IT(&hspi_SPI,&aRxBuffer[i],7);
  #if 0
  HAL_SPI_Transmit(&hspi_SPI,&SPI_aTxBuffer[0],7,0XFFFF);
  {
   if(aRxBuffer[0]==0xa2)
	
	 {
		printf("%#x\n",aRxBuffer[0]);  
		printf("%#x\n",aRxBuffer[1]);  
		printf("%#x\n",aRxBuffer[2]);  
		printf("%#x\n",aRxBuffer[3]);  
		printf("%#x\n",aRxBuffer[4]);  
		printf("%#x\n",aRxBuffer[5]);
		printf("%#x\n",aRxBuffer[6]);
	 }

	}
  
  #endif
  
  HAL_SPI_Transmit_IT(&hspi_SPI,&SPI_aTxBuffer[0],7);
	
}
#endif
/***************************************************************
*
*��������
*�������ܣ�I2C�жϽ��ջص�����
*��������
*����ֵ����
*
****************************************************************/
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
         I2C_RX_SAVE_Buffer[j]=i2c_rx_data;
		 if(j==0)
		 {
		     j++;
			 printf("rxcpltcall[0]��%#x\n",i2c_rx_data);
		 }
		 else if(j==1)
		 {
             j++;
			 printf("rxcpltcall[1]��%#x\n",i2c_rx_data);
			 
		 }
		 else if(j==2)
		 {
		   j=0;
		   printf("rxcpltcall[2]��%#x\n",i2c_rx_data);
		 }
		 HAL_I2C_Slave_Receive_IT(&I2cHandle,&i2c_rx_data,4);
	
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
