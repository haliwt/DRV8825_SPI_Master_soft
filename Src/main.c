
/* °üº¬Í·ÎÄ¼ş ----------------------------------------------------------------*/
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

#define SENDBUFF_SIZE             100 // ´®¿ÚDMA·¢ËÍ»º³åÇø´óĞ¡
#define STEPMOTOR_MICRO_STEP      32  // ²½½øµç»úÇı¶¯Æ÷Ï¸·Ö£¬±ØĞëÓëÇı¶¯Æ÷Êµ¼ÊÉèÖÃ¶ÔÓ¦

/* Ë½ÓĞ±äÁ¿ ------------------------------------------------------------------*/
uint8_t aRxBuffer[256];                      // ½ÓÊÕÊı¾İ 
uint8_t aTxBuffer[SENDBUFF_SIZE];       // ´®¿ÚDMA·¢ËÍ»º³åÇø
uint8_t repcdata[3]={0x00,0x00,0x00};                    //´ÓÉÏÎ»»ú½ÓÊÕµ½µÄ3¸ö×Ö½ÚÊı¾İ¡£
extern uint8_t SPI_aRxBuffer[7];
extern uint8_t SPI_aTxBuffer[7];
uint8_t I2C_RX_Buffer[3];  //I2C ½ÓÊÕµ½µÄÊı¾İ
uint8_t I2C_RX_SAVE_Buffer[3];
/* À©Õ¹±äÁ¿ ------------------------------------------------------------------*/
extern __IO uint16_t Toggle_Pulse; /* ²½½øµç»úËÙ¶È¿ØÖÆ£¬¿Éµ÷½Ú·¶Î§Îª 650 -- 3500 £¬ÖµÔ½Ğ¡ËÙ¶ÈÔ½¿ì */
__IO uint32_t pulse_count; /*  Âö³å¼ÆÊı£¬Ò»¸öÍêÕûµÄÂö³å»áÔö¼Ó2 */
extern  __IO uint16_t  home_position    ; 
extern __IO uint8_t Brightness;   
extern __IO uint8_t save_flag;
__IO uint8_t re_intrrupt_flag=0; //´ÓÉÏÎ»»ú£¬½ÓÊÕĞÅºÅ±êÖ¾Î»£¬½øÈëÖĞ¶Ï±êÖ¾Î»
__IO uint16_t judge_data;    //½ÓÊÕµ½Êı¾İµÄ¹¦ÄÜÂëÊı¾İ¡£
extern __IO uint8_t key_stop;
__IO uint8_t PB8_flag=0;
extern __IO uint8_t stop_key_flag;
uint8_t SPI_TX_FLAG=0;
extern uint8_t test_aTxBuffer[7];
uint8_t i2c_rx_data;
static uint8_t j=0;
extern __IO uint8_t END_STOP_FLAG;  //Âí´ïÔËĞĞµ½ÖÕµã£¬Í£Ö¹±êÖ¾Î
__IO uint8_t A2_FLAG=0;
__IO uint8_t A1_Read_FLAG=0; //A1 è¯»å–å‚æ•°å€¼

/* À©Õ¹±äÁ¿ ------------------------------------------------------------------*/
/* Ë½ÓĞº¯ÊıÔ­ĞÎ --------------------------------------------------------------*/
/* º¯ÊıÌå --------------------------------------------------------------------*/
/**
  * º¯Êı¹¦ÄÜ: ÏµÍ³Ê±ÖÓÅäÖÃ
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // Íâ²¿¾§Õñ£¬8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9±¶Æµ£¬µÃµ½72MHzÖ÷Ê±ÖÓ
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ÏµÍ³Ê±ÖÓ£º72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBÊ±ÖÓ£º72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1Ê±ÖÓ£º36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2Ê±ÖÓ£º72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

 	// HAL_RCC_GetHCLKFreq()/1000    1msÖĞ¶ÏÒ»´Î
	// HAL_RCC_GetHCLKFreq()/100000	 10usÖĞ¶ÏÒ»´Î
	// HAL_RCC_GetHCLKFreq()/1000000 1usÖĞ¶ÏÒ»´Î
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // ÅäÖÃ²¢Æô¶¯ÏµÍ³µÎ´ğ¶¨Ê±Æ÷
 // HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100000);  // ÅäÖÃ²¢Æô¶¯ÏµÍ³µÎ´ğ¶¨Ê±Æ÷
  /* ÏµÍ³µÎ´ğ¶¨Ê±Æ÷Ê±ÖÓÔ´ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* ÏµÍ³µÎ´ğ¶¨Ê±Æ÷ÖĞ¶ÏÓÅÏÈ¼¶ÅäÖÃ */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/*********************************************************
  * º¯Êı¹¦ÄÜ: Ö÷º¯Êı.
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ   SPI1 Í¨ĞÅ´Ó»ú
  *********************************************************/
int main(void)
{
  uint8_t txbuf[100],Mode_Count=0;
  // uint8_t tranbuffer[]={0x88};
   uint8_t DS18B20ID[8],i;//rxspidata;
   
  /* ¸´Î»ËùÓĞÍâÉè£¬³õÊ¼»¯Flash½Ó¿ÚºÍÏµÍ³µÎ´ğ¶¨Ê±Æ÷ */
  HAL_Init();
  /* ÅäÖÃÏµÍ³Ê±ÖÓ */
  SystemClock_Config();
  LED_GPIO_Init();
  KEY_GPIO_Init();
  GENERAL_TIMx_Init();
	
  MX_USARTx_Init();
  STEPMOTOR_TIMx_Init();
  MX_I2C_EEPROM_Init();
  /* È·¶¨¶¨Ê±Æ÷ */
  SPIx_Init();
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);

  // __HAL_UART_ENABLE_IT(&husartx, UART_IT_IDLE);  //wt.edit 11.07
  memcpy(txbuf,"SPI_MASTER version6.01 2018-04-07 \n",100);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  Brightness=LAMP_Read_BrightValue(); 
  GENERAL_TIMx_Init();
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4); 
  /* Ê¹ÄÜ½ÓÊÕ£¬½øÈëÖĞ¶Ï»Øµ÷º¯Êı */
  HAL_UART_Receive_IT(&husartx,aRxBuffer,7);
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_1);//HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_1);
   /* Ê¹ÄÜ½ÓÊÕ£¬½øÈëÖĞ¶Ï»Øµ÷º¯Êı */
  //Brightness=LAMP_Read_BrightValue(); 
  GENERAL_TIMx_Init();
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4);  
   printf("DS18B20ÎÂ¶È´«¸ĞÆ÷ĞÅÏ¢¶ÁÈ¡\n");
  while(DS18B20_Init())	
  {
		printf("DS18B20ÎÂ¶È´«¸ĞÆ÷²»´æÔÚ\n");    
        HAL_Delay(1000);
	    break;
  }
  printf("¼ì²âµ½DS18B20ÎÂ¶È´«¸ĞÆ÷£¬²¢³õÊ¼»¯³É¹¦\n");
  DS18B20_ReadId(DS18B20ID);
  printf("DS18B20µÄĞòÁĞºÅÊÇ£º 0x");  
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
		
	     DRV8825_SLEEP_DISABLE() ; //¸ßµçÆ½¿ªÊ¼¹¤×÷
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
				 	//STEPMOTOR_AxisHome(0x03); //ËÑË÷Ô­µã
				}break;
				case 1://CWË³Ê±Õë·½ÏòĞı×ªn*SPR±íÊ¾Ğı×ª3È¦
				{
					STEPMOTOR_AxisMoveRel(1*SPR, Toggle_Pulse);
				}break;
				case 2://CCWÄæÊ±Õë·½ÏòĞı×ª
				{
						STEPMOTOR_AxisMoveRel(-1*SPR, Toggle_Pulse);
				}break;
				case 3://»Ø×ø±êÔ­µã¹¦ÄÜ,×Ô¶¯»Øµ½×ø±ê
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
		if( END_STOP_FLAG==1)  //ÔËĞĞµ½ÖÕµã£¬Í£Ö¹±êÖ¾Î»¡£
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
  * º¯Êı¹¦ÄÜ: ´®¿Ú½ÓÊÕÍê³É»Øµ÷º¯Êı
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷£ºÎŞ
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
								repcdata[0]=	aRxBuffer[3]; //×î¸ß×Ö½Ú
							    repcdata[1]=	aRxBuffer[4]; //ÖĞ¼ä×Ö½Ú
							    repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
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
								   repcdata[0]=	aRxBuffer[3]; //×î¸ß×Ö½Ú
								   repcdata[1]=	aRxBuffer[4]; //ÖĞ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x02;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 
								}
							
							break;
						case 0x82 :
							if(aRxBuffer[6]==0x0b)
								{
									re_intrrupt_flag=1;
									repcdata[0]=	aRxBuffer[3]; //×î¸ß×Ö½Ú
								   repcdata[1]=	aRxBuffer[4]; //ÖĞ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x82;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
								}
							
							break;
						case 0x90 :
							if(aRxBuffer[6]==0x0b)
								{	  
									re_intrrupt_flag=1;
								   repcdata[1]=	aRxBuffer[4]; //ÖĞ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x90;
									
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							
							break;
						case 0xa0 :
							if(aRxBuffer[6]==0x0b)   //ÖØĞÂÉèÖÃÔ­µã
								{
									//DRV8825_SLEEP_DISABLE(); //¸ßµçÆ½¿ªÊ¼¹¤×÷,½â³ıĞİÃß×´Ì¬
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
		/*************************A1è¯»å–é©¬è¾¾å‚æ•°å€¼********************************/
		   if(aRxBuffer[1]==0x01)   
		   {
		     switch(aRxBuffer[2])
			 {
				 case 0x03:   //¶ÁÈ¡Âí´ïÊµÊ±Î»ÖÃÂö³åÊı
				    if(aRxBuffer[6]==0x0b)
						{
						   A1_Read_FLAG=1;
						   judge_data= 0x103;
						  //Display_CurrentPosition();
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
				  break; 
				case 0xe0:   //è¯»å–A1 EEPROM å€¼
				    if(aRxBuffer[6]==0x0b)
						{
						   A1_Read_FLAG=1;
						   judge_data= 0x1e0;
						   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
				  break; 
				 case 0x04:      /* ¶ÁÈ¡LED µÆÁÁ¶ÈÖµ */
					 {
						if(aRxBuffer[6]==0x0b)
						{

							A1_Read_FLAG=1;
						   judge_data= 0x104;
							
						
						__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
				    break;
				  case 0x01 ://case 0x01 :     /*if(aRxBuffer[2]==0x01)¶ÁÈ¡Âí´ï×ªËÙÖµ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{

							A1_Read_FLAG=1;
						   judge_data= 0x101;
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
					 break;
					 case 0x02:    //if(aRxBuffer[2]==0x02)  /*¶ÁÈ¡»·¾³ÎÂ¶ÈÖµ*/
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

  /*****************æ§åˆ¶ç¬¬äºŒä¸ªé©¬è¾¾æŒ‡ä»¤******************************/
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
								repcdata[0]=	aRxBuffer[3]; //×î¸ß×Ö½Ú
							    repcdata[1]=	aRxBuffer[4]; //ÖĞ¼ä×Ö½Ú
							    repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
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
								   repcdata[0]=	aRxBuffer[3]; //×î¸ß×Ö½Ú
								   repcdata[1]=	aRxBuffer[4]; //ÖĞ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x202;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 
								}
							
							break;
						case 0x82 :
							if(aRxBuffer[6]==0x0b)
								{
									A2_FLAG=1;
								   repcdata[0]=aRxBuffer[3]; //×î¸ß×Ö½Ú
								   repcdata[1]=	aRxBuffer[4]; //ÖĞ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x282;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
								}
							
							break;
						case 0x90 :
							if(aRxBuffer[6]==0x0b)
								{	  
									A2_FLAG=1;
								   repcdata[1]=	aRxBuffer[4]; //ÖĞ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x290;
									
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							
							break;
						case 0xa0 :
							if(aRxBuffer[6]==0x0b)   //ÖØĞÂÉèÖÃÔ­µã
								{
									//DRV8825_SLEEP_DISABLE(); //¸ßµçÆ½¿ªÊ¼¹¤×÷,½â³ıĞİÃß×´Ì¬
									 A2_FLAG=1;
									 judge_data= 0x2a0;
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
									
								}
							break;
						case 0x00 :   //¿ØÖÆµÚ¶şÂí´ï£¬Í£Ö¹¡£
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
						case 0xd0 :  //Çå³şEEPROMÊı¾İ
							 if(aRxBuffer[6]==0x0b)
								{
								   A2_FLAG=1;
								   judge_data= 0x2d0;
									 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
								}
							  break;
					case 0xee:   //²âÊÔA1 ÓëA2 °åÍ¨Ñ¶Í¬²½¡£A1,A2 LEDÍ¬Ê±ÉÁË¸
							 A2_FLAG=1;
							  judge_data= 0x2ee;
						       __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
							  break;
						
			    }
			} //end if(aRxBuffer[1]==0x00)
		   /*****************************¶ÁÈ¡µÚ¶şÂí´ïÖ¸Áî**********************************/
		   if(aRxBuffer[1]==0x01)   
		   {
		     switch(aRxBuffer[2])
			 {
				 case 0x03:   //¶ÁÈ¡Âí´ïÊµÊ±Î»ÖÃÂö³åÊı
				    if(aRxBuffer[6]==0x0b)
						{
						   A2_FLAG=1;
						   judge_data= 0x2103;
						  //Display_CurrentPosition();
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
				  break; 
				 case 0x04:      /* ¶ÁÈ¡LED µÆÁÁ¶ÈÖµ */
					 {
						if(aRxBuffer[6]==0x0b)
						{

							A2_FLAG=1;
						   judge_data= 0x2104;
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
				    break;
				  case 0x01 ://case 0x01 :     /*if(aRxBuffer[2]==0x01)¶ÁÈ¡Âí´ï×ªËÙÖµ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{
                          A2_FLAG=1;
						   judge_data= 0x2101;
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
					 break;
					 case 0x02:    //if(aRxBuffer[2]==0x02)  /*¶ÁÈ¡»·¾³ÎÂ¶ÈÖµ*/
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
*º¯Êı¹¦ÄÜ£ºSPIÖĞ¶Ï£¬»Øµ÷º¯Êı
*²ÎÊı£ºÎŞ
*·µ»ØÖµ£ºÎŞ
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
*º¯ÊıÃû£º
*º¯Êı¹¦ÄÜ£ºI2CÖĞ¶Ï½ÓÊÕ»Øµ÷º¯Êı
*²ÎÊı£ºÎŞ
*·µ»ØÖµ£ºÎŞ
*
****************************************************************/
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
         I2C_RX_SAVE_Buffer[j]=i2c_rx_data;
		 if(j==0)
		 {
		     j++;
			 printf("rxcpltcall[0]£º%#x\n",i2c_rx_data);
		 }
		 else if(j==1)
		 {
             j++;
			 printf("rxcpltcall[1]£º%#x\n",i2c_rx_data);
			 
		 }
		 else if(j==2)
		 {
		   j=0;
		   printf("rxcpltcall[2]£º%#x\n",i2c_rx_data);
		 }
		 HAL_I2C_Slave_Receive_IT(&I2cHandle,&i2c_rx_data,4);
	
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
