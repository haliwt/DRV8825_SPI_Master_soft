#include "DRV8825/drv8825.h"
#include "led/bsp_led.h"
#include "StepMotor/bsp_StepMotor.h"
#include "i2c/bsp_EEPROM.h"
#include "key/bsp_key.h"
#include "hextodec/hextodec.h"
#include "usart/bsp_usartx.h"
#include "lamp/bsp_lamp.h"
#include "GeneralTIM/bsp_GeneralTIM.h"

extern uint8_t I2C_RX_SAVE_Buffer[3];
extern uint8_t repcdata[3]; //上传给上位机的数据
extern __IO uint16_t Toggle_Pulse ;
extern __IO uint32_t pulse_count; /*  脉冲计数，一个完整的脉冲会增加2 */
extern uint8_t I2c_Buf_Write[256];
extern uint8_t I2c_Buf_Read[256];

extern speedRampData srd  ;
static __IO int32_t  step_position=0 ;
__IO uint8_t stop_flag=0;
extern __IO uint32_t PulseNumbers;
extern __IO uint32_t step_count; 
__IO uint8_t home_flag=0; 
extern __IO uint32_t step_count;  //相PC机发送存储的马达脉冲数
__IO uint8_t NewOrigin_flag=0;
extern __IO uint8_t back_flag;
extern __IO int32_t  LimPosi ; //正方向极限标志位  True:到达极限位  False:未到达极限位
extern __IO int32_t  LimNega ; //负方向极限标志位
extern __IO uint8_t END_STOP_FLAG;  //马达运行到终点，停止标志位

/***************************************
*
*函数功能:马达速度设置
*参数：设定马达速度
*DRV8825 nENBL --- Hight level is diable ,
               ----Low leve is enable
****************************************/
void DRV8825_SetSpeed(uint8_t high_b,uint8_t low_b)
{
	  uint8_t i,flag;
	  flag=EEPROM_CheckOk();
	   if(flag==1)
	   {
     // for ( i=0; i<2; i++ ) //填充缓冲 ,
      {
      I2c_Buf_Read[0]=0;      // 清空接收缓冲区
      I2c_Buf_Read[1]=0; 
      I2c_Buf_Write[0] = high_b;   // 为发送缓冲区填充数据
	  I2c_Buf_Write[1] = low_b;
      }
     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
	 EEPROM_WriteBytes(I2c_Buf_Write, 16, 2);  
     HAL_Delay(100);
	
     //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2); 
     for (i=0;i<2;i++)
     {    
      if(I2c_Buf_Read[i] != I2c_Buf_Write[i])
      {
        LED1_ON;
		LED2_ON;
		while(1); //wt.edit
      }
     }
    if(i==2) //表示写入成功
        {
        LED1_ON;
		LED2_ON;
		HAL_Delay(200);
		LED1_OFF;
		LED2_OFF;
    	}
	}	
	
}	
/*******************************************
*
*函数功能：读取速度值
*参数：无
*返回值：读取到的速度值 ,存放在数组,[2][3]
*
********************************************/
uint16_t DRV8825_ReadSpeed(void)
{
    uint8_t flag;
	  uint16_t temp;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
       	 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //地址是 0x0F,字节数是2
		 temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
		
     	 }
		return temp;
    
}

/****************************************
*
*函数功能：相对位置移动，以当前的位置计算。
*参数: 输入的16进制，2个字节?---脉冲数驱动
*返回值：无，背离马达位置方向移动     0x82---往前移动
*
***************************************/
void DRV8825_CCW_AxisMoveRel(uint8_t ccir_high,uint8_t ccir_mid,uint8_t ccir_low,uint16_t speed)
{
   uint32_t all_cir;
   uint32_t temp4;
	 all_cir=ccir_high<<16|ccir_mid<<8|ccir_low;
	// STEPMOTOR_AxisMoveRel(1*all_cir*SPR,speed);
     if(stop_flag==0)    //第一次开机，执行此语句  wt.edit 2018.04.13
	{
	  
	  stop_flag=35;
	  temp4=DRV8825_Read_CurrentPosition();   //负数--远离马达方向移动
	  step_count=temp4;
	  PulseNumbers=temp4;
	  step_position=temp4;
	
	}
       
	 STEPMOTOR_AxisMoveRel(1*all_cir,speed);   

}

/****************************************
*
*函数功能：相对位置移动，以当前的位置计算。
*参数: 输入的16进制，3个字节，以脉冲数驱动
*返回值：无,向马达位置方向移动------order:0x02
*
***************************************/
void DRV8825_CW_AxisMoveRel(uint8_t cir_high,uint8_t cir_mid,uint8_t cir_low,uint16_t speed)
{
     int32_t all_cir;
     uint32_t temp4;
	 all_cir=cir_high<<16|cir_mid<<8|cir_low;
	 //STEPMOTOR_AxisMoveRel(-1*all_cir*SPR,speed); 
    if(stop_flag==0)    //第一次开机，执行此语句  wt.edit 2018.04.13
	{
	  
	  stop_flag=35;
	  temp4=DRV8825_Read_CurrentPosition();   //负数--远离马达方向移动
	  step_count=temp4;
	  PulseNumbers=temp4;
	  step_position=temp4;
	
	}
	 STEPMOTOR_AxisMoveRel(-1*all_cir,speed); 

}
/**********************************
*
*函数功能：马达停止
*
**********************************/
void DRV8825_StopMove(void)
{
       
        stop_flag=1;
	   // PulseNumbers=0;  //
        HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
       // 关闭通道
       TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);        
       __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
      // DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_SLEEP_ENABLE(); //进入省电模?
       DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_OUTPUT_DISABLE();    //DRV8825芯片高电平，没有输出?
       Display_EEPROM_Value();

}

/****************************************************************
*
*函数功能：存储马达当前的位置,step_position
*参数：无
*返回值：无
*
****************************************************************/
void DRV8825_Save_CurrentPosition(void)
{
     uint8_t flag,flag_w,i;
     uint32_t a,b,c,d;

	 a=step_count & 0xff;        //最高位
     b=step_count >>8 & 0xff;   //第二位
     c=step_count >>16 & 0xff;  //第三位
     d=step_count >>24 & 0xff;  //第四位,最低位
	
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<4; i++ ) //填充缓冲 ,
	      {
	      I2c_Buf_Read[i]=0;      // 清空接收缓冲区
	      I2c_Buf_Write[i]=0;
	      
	      }
	      I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     
      
	     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 5, 4); //0x05 ~0x09
		 HAL_Delay(200);
		 #if 0
		  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4); 
	   if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
		 {
		    if(I2c_Buf_Read[1] == I2c_Buf_Write[1]) //
				if(I2c_Buf_Read[2] == I2c_Buf_Write[2]) //
				if(I2c_Buf_Read[3] == I2c_Buf_Write[3]) //
					{
						LED1_ON;
						LED2_ON;
						printf("eeprom is save OK \n");
						//while(1);
					}
		 
		 }
		 else printf("eeprom is save error \n");
		 #endif 
	
     //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	
	     if(flag_w==1) //表示写入成功
	        {
	        LED1_ON;
			HAL_Delay(10);
			LED1_OFF;
		    }
		   else
         printf("eeprom check error \n");				 
		   	//while(1);
	}	 

}
/****************************************************************
*
*函数功能：读取马达当前位置马达当前的位置,step_position
*参数：无
*返回值：无 I2c_Buf_Read[0]=MSB    I2c_Buf_Read[3]=LSB
*
****************************************************************/
uint32_t DRV8825_Read_CurrentPosition(void)
{
       uint8_t flag;
	   uint32_t result,temp1,temp2,temp3,temp4;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
       	 EEPROM_ReadBytes(I2c_Buf_Read, 5, 4); 
		  HAL_Delay(200);
		 //temp=(0xff & I2c_Buf_Read[3]<<24)|(0xff & I2c_Buf_Read[2]<<16)|(0xff & I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0] & 0xff);
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
         temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
         temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
         temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
		
	   // HAL_UART_Transmit(&husartx,I2c_Buf_Read,4,5);
		result=temp1+temp2+temp3+temp4;
		
     	}
		return result;
     
}
/*********************************************************
 *
 * 函数功能：显示一个当前马达位置的参数函数。 
 * 输入参数：无
 * 返回参数无：
 *
***********************************************************/
void Display_EEPROM_Value(void)
{
       int32_t temp1,temp2,temp3,temp4;
	   uint8_t flag,i;
	   uint32_t real_value,step_numbers;
	   uint8_t sendbuffer[6]={0xa1,0x03,00,00,00,0x0b};
	   real_value=step_count;
      
       
       if(stop_flag==0)
       {
       
         flag=EEPROM_CheckOk();
	     if(flag==1)
		 {
    	  for(i=0;i<4;i++)
		  {
		    I2c_Buf_Read[i]=0;
		  }
		  /*??EEPROM??????????????I2c_Buf_Read?? */ 
       	  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //马达走的位置的数据，存储位置
		  HAL_Delay(200);

          sendbuffer[4]=I2c_Buf_Read[3];
	      sendbuffer[3]=I2c_Buf_Read[2];
	      sendbuffer[2]=I2c_Buf_Read[1];
		  
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
	     temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
	     temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
	     temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
		 real_value=temp1+temp2+temp3+temp4;
		 
		  
		// temp=(I2c_Buf_Read[3]<<24)|(I2c_Buf_Read[2]<<16)|(I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0]);
      
	      printf("real position= %ld \n",real_value);
         HAL_UART_Transmit(&husartx,sendbuffer,6,12);		
       
         }
        }
       else 
       {
           sendbuffer[4]=real_value & 0xff;
           sendbuffer[3]=real_value>>8 & 0xff;
           sendbuffer[2]=real_value>>16 & 0xff;
           
           step_numbers=step_position;   
           
           printf("real position= %ld \n",real_value);
           printf("step_numbers= %d \n",step_numbers);
           HAL_UART_Transmit(&husartx,sendbuffer,6,12);  
       } 
		
}
/*********************************************************
 *
 * 函数功能：读取A1 EEPROM值
 * 输入参数：无
 * 返回参数无：
 *
***********************************************************/
void A1_ReadEprom_Value(void)
{
       int32_t temp1,temp2,temp3,temp4;
	   uint8_t flag,i;
	   uint8_t sendbuffer[6]={0xa1,0x03,00,00,00,0x0b};
	   uint32_t value;
	   
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	  for(i=0;i<4;i++)
		  {
		    I2c_Buf_Read[i]=0;
		  }
		  /*??EEPROM??????????????I2c_Buf_Read?? */ 
       	  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //马达走的位置的数据，存储位置
		  HAL_Delay(200);

          sendbuffer[4]=I2c_Buf_Read[3];
	      sendbuffer[3]=I2c_Buf_Read[2];
	      sendbuffer[2]=I2c_Buf_Read[1];
		  
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
	     temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
	     temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
	     temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
		 value=temp1+temp2+temp3+temp4;
		 
		  
		// temp=(I2c_Buf_Read[3]<<24)|(I2c_Buf_Read[2]<<16)|(I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0]);
      
	     printf("EEPROM position= %ld \n",value);
         HAL_UART_Transmit(&husartx,sendbuffer,6,12);		

      }
  }
/*********************************************************
 *
 * 函数名称：A2_Pulse_RealTime_Value
 * 函数功能：传给上位机第二马达实时脉冲数。
 * 输入参数：无 --负数是补码上传
 * 返回参数无：
 *
***********************************************************/
void A2_Pulse_RealTime_Value(void)
{
       uint32_t temp2,temp3,temp4;
	   uint8_t sendbuffer[6]={0xa2,0x03,00,00,00,0x0b};
	   int32_t value;
	   
	   #if 0
	   if(I2C_RX_SAVE_Buffer[0]==0xff)
	   {
	       
			       repcdata[0]= ~I2C_RX_SAVE_Buffer[0]+1;
				   repcdata[1]= ~I2C_RX_SAVE_Buffer[1]+1;
				   repcdata[2]= ~I2C_RX_SAVE_Buffer[2]+1;
				   
				       
				   
				     printf("sendbuffer[4]= %#x \n",I2C_RX_SAVE_Buffer[2]);
				     printf("sendbuffer[3]= %#x \n",I2C_RX_SAVE_Buffer[1]);
				     printf("sendbuffer[2]= %#x \n",I2C_RX_SAVE_Buffer[0]);
		   
		              temp2=Hex2oct_MD1(I2C_RX_SAVE_Buffer[0]);
					  temp3=Hex2oct_MD2(I2C_RX_SAVE_Buffer[1]);
					  temp4=Hex2oct_LSB(I2C_RX_SAVE_Buffer[2]);
					  value=temp2+temp3+temp4;
				   
				     printf("RealTime=-%#x \n",value);	
		             HAL_UART_Transmit(&husartx,sendbuffer,6,12);	
                    			   
			   
		}
	   else 
	  #endif 
	   {
	    
	     
		 
		 sendbuffer[4]=I2C_RX_SAVE_Buffer[2];
	     sendbuffer[3]=I2C_RX_SAVE_Buffer[1];
	     sendbuffer[2]=I2C_RX_SAVE_Buffer[0];
		   
		temp2=Hex2oct_MD1(I2C_RX_SAVE_Buffer[0]);
	    temp3=Hex2oct_MD2(I2C_RX_SAVE_Buffer[1]);
		temp4=Hex2oct_LSB(I2C_RX_SAVE_Buffer[2]);
		value=temp2+temp3+temp4;
	   
	     printf("RealTime= %ld \n",value);
		 HAL_UART_Transmit(&husartx,sendbuffer,6,12);		
	   }
}

/*********************************************************
 *
 * 函数名称：A2_Pulse_EEPROM_Value
 * 函数功能：传给上位机第二马EEPROM of value
 * 输入参数：无
 * 返回参数无：
 *
***********************************************************/
void PC_A2_Pulse_EEPROM_Value(void)
{
       uint32_t temp2,temp3,temp4;
	   uint8_t sendbuffer[6]={0xa2,0xe0,00,00,00,0x0b};
	   int32_t value;
	   
	   sendbuffer[4]=I2C_RX_SAVE_Buffer[2];
	   sendbuffer[3]=I2C_RX_SAVE_Buffer[1];
	   sendbuffer[2]=I2C_RX_SAVE_Buffer[0];
	  
	   
	     
	     temp2=Hex2oct_MD1(sendbuffer[2]);
	     temp3=Hex2oct_MD2(sendbuffer[3]);
	     temp4=Hex2oct_LSB(sendbuffer[4]);
	     value=temp2+temp3+temp4;
		  
		printf("EEPROM position= %d \n",value);
        HAL_UART_Transmit(&husartx,sendbuffer,6,12);	 
}


/******************************************
*
*函数功能：清楚EEPROM的存储器的值位0
*
************************************************/
void EEPROM_Clear_Buf(void)
{
 uint16_t i;
  uint8_t flag,flag_w;
  flag=EEPROM_CheckOk();
  
  if(flag==1)
  {
    for ( i=0; i<10; i++ ) //??仺?? ,
      {
      I2c_Buf_Read[i]=0;      // ???????????
      I2c_Buf_Write[i]=0;      
      }
     //??I2c_Buf_Write??????????????д??EERPOM?? 
	 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 9); //
	 step_count=0; //wt.edit 2018.02.23
	 HAL_Delay(100);
  }
  if(flag_w==1)
 {
   LED1_ON;
   HAL_Delay(20);
   LED1_OFF;
 }
  else while(1);

}

/****************************************************************
 *
 *函数功能:存储当前马达的位置,在EEPROM中的存储位置是:0~3-4个字节
 *输入参数：无
 *返回值：无
 *
****************************************************************/
void MOTOR_Save_NewHomePosition(void)
{
     uint8_t flag,flag_w,i;
     uint8_t a,b,c,d;
     a=PulseNumbers & 0xff;        //???λ
     b=PulseNumbers >>8 & 0xff;   //???λ
     c=PulseNumbers >>16 & 0xff;  //????λ
     d=PulseNumbers >>24 & 0xff;  //????λ,???λ
	
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
          for ( i=0; i<4; i++ ) //清除I2C_Buf_Read,and write
	      {
		      I2c_Buf_Read[i]=0;      // ???????????
		      I2c_Buf_Write[i]=0;      
	      }
		  I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     //??I2c_Buf_Write??????????????д??EERPOM?? 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 4); //位置是 0x00～0x03，四个字节
		 HAL_Delay(100);
		 #if 0
		 EEPROM_ReadBytes(I2c_Buf_Read, 4, 4); 
	     if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
		 {
		    if(I2c_Buf_Read[1] == I2c_Buf_Write[1]) //
				if(I2c_Buf_Read[2] == I2c_Buf_Write[2]) //
					if(I2c_Buf_Read[3] == I2c_Buf_Write[3]) //
					{
						LED1_ON;
						LED2_ON;
						while(1);
					}
		
		
	      }
		  #endif 
		if(flag_w==1) //???д????
        {
        LED1_ON;
		LED2_ON;
		HAL_Delay(200);
		LED1_OFF;
		LED2_OFF;
    	}
		else 
		while(1);
     }
     else while(1);	  
	 
 }	 


/****************************************************************
 *
 *函数功能:存储马达设置原点的标志位，home_flag=1,
 *         home_flag==1,表示马达设置了新的原点,储存在EEPROM
           中的地址 0x04
 *输入参数：无
 *返回值：1---表示马达重新设置新的原点坐标。
 *
****************************************************************/
uint8_t MOTOR_NewHome_FalgSave(void)
{
     uint8_t flag,flag_w;
  
     flag=EEPROM_CheckOk();
	 if(flag==1)
	 {
	   I2c_Buf_Read[0]=0;      // ???????????
       I2c_Buf_Write[0]=1;      
      
    
	 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 4, 1); //地址：0x04,写入一个数据
	 HAL_Delay(100);
	 #if 0  //测试写入的数据
	 EEPROM_ReadBytes(I2c_Buf_Read, 4, 4); 
     if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
	 {
	   LED1_ON;
	   LED2_ON;
	  while(1);
				
	 }
	 #endif 
	 }
     //??EEPROM??????????????I2c_Buf_Read??  
   	
     if(flag_w==1) //???д????
        {
        LED1_TOGGLE ;
		return 1;
    	}
	 else 
	 	return 0;
}	 


/******************************************************
 *
 *函数功能：是读取冲设置原点的标志位：home_flag
 *输入参数无：
 *返回参数： 1 ---表示有重新设置原点成功， 0----表示没有重新设置原点-失败。
 *
******************************************************/
uint8_t MOTOR_Read_NewHomeFlag(void)
{
       uint8_t flag;
	   uint32_t result;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 
        I2c_Buf_Read[0]=0;
		  EEPROM_ReadBytes(I2c_Buf_Read, 4, 1); //从地址：0x04,读一个字节。 
		  HAL_Delay(200);  
          result=I2c_Buf_Read[0];
		  LED2_TOGGLE ;
		 
     	}
		 return result;
}
/***************************************************
 *
 *函数功能：设置新的坐标原点
 *
 *
*******************************************************/
void Set_NewOrigin_Position(void)
{
    uint8_t flag,flag_w,i;
	NewOrigin_flag=1;   //wt.edit 2018.04.11 add this is item
    PulseNumbers=0;
    step_count=0;
	step_position=0;  //wt.edit 18.03.07
    flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<10; i++ ) //娓呮 EEPROM 瀛樺偍鐨刡uffer娓呴浂
	      {
	      I2c_Buf_Read[i]=0;      // 清空接收缓冲区
	      I2c_Buf_Write[i]=0;
	      }
	     
     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 10); //0x05 ~0x09
		 HAL_Delay(50);
	   //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	 DRV8825_Save_CurrentPosition();
	  }	 
		 if(flag_w==1) //表示写入成功
	    {
	        LED1_ON;
			HAL_Delay(10);
			LED1_OFF;
		   }
		   else 
		   	  LED1_ON;
			  LED2_ON;
			  HAL_Delay(500);
			  LED1_OFF;
			  LED2_OFF;
			  HAL_Delay(500);
			  LED1_ON;
			  LED2_ON;
			   HAL_Delay(500);
			  LED1_OFF;
			  LED2_OFF;


}

/*******************************************
*
*函数功能：上位机读取马达的速度值
*参数：无
*返回值：读取到的速度值 ,存放在数组,[2][3]
*
********************************************/
void PC_DRV8825_ReadSpeed(void)
{
		  uint8_t flag;
		  uint16_t temp;
		  uint8_t sendbuffer[6]={0xa1,0x01,00,00,00,0x0b};
		   flag=EEPROM_CheckOk();
		   if(flag==1)
			 {
			 //将EEPROM读出数据顺序保持到I2c_Buf_Read中	
			 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //地址是 0x0F,字节数是2
			 temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
			 }
		    sendbuffer[4]=I2c_Buf_Read[1];
	        sendbuffer[3]=I2c_Buf_Read[0];
	       // sendbuffer[2]=I2c_Buf_Read[1]; 
		   printf("Motor Speed is : %d \n",temp);
		   HAL_UART_Transmit(&husartx,sendbuffer,6,12);	
			
}
/*******************************************
*
*函数功能：读取第二个马达速度值 A2
*参数：无
*返回值：读取到的速度值 ,存放在数组,[2][3]
*
********************************************/
void PC_A2_DRV8825_ReadSpeed(uint8_t speed_hig,uint8_t speed_low)
{
          uint16_t temp;
		  uint8_t sendbuffer[6]={0xa2,0x01,00,00,00,0x0b};
		  sendbuffer[4]=speed_low;
	      sendbuffer[3]=speed_hig;
	       temp=speed_hig<<8|speed_low;
		   printf("Motor Speed is : %d \n",temp);
		   HAL_UART_Transmit(&husartx,sendbuffer,6,12);	


}
/********************************************************************************************************************
*
*函数功能：马达移动的绝对位置坐标函数 ---逆时针
*参数：1，2，3 共3个字节数，---脉冲数 0x01
*返回函数：无
*
*********************************************************************************************************************/
void STEPMOTOR_CCW_AxisMoveAbs( uint8_t ab_h,uint8_t ab_m,uint8_t ab_low, uint32_t speed) //绝对位置移动函数-逆时针
{
   
	 int32_t ccw_targert_step;
	 
	ccw_targert_step=step_position -(ab_h<<16|ab_m<<8|ab_low);
    STEPMOTOR_AxisMoveRel(-1*ccw_targert_step ,speed); 
   

}
/********************************************************************************************************************
*
*函数功能：马达移动的绝对位置坐标函?--?--顺时针
*参数：1，2，3 共3个字节数，---脉冲数 0x01
*返回函数：无
*
*********************************************************************************************************************/
 void STEPMOTOR_CW_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed) //绝对位置移动函数-逆时针
 {
       
         int32_t cw_targert_step;
	   
        cw_targert_step=step_position - (int32_t)(abs_high<<16|abs_mid<<8|abs_low);
        STEPMOTOR_AxisMoveRel(1*cw_targert_step,speed);
    
   }

/****************************************************************
 *
 *函数名称：
 *函数功能：存储马达到终点的位置，不清零，自动存取
 *参数：无
 *返回值：无
 *
****************************************************************/
void Motor_Save_EndPosition(void)
{
     uint8_t flag,flag_w,i;
     uint32_t a,b,c,d;
   
	
	 a=step_count & 0xff;        //最高位
     b=step_count >>8 & 0xff;   //第二位
     c=step_count >>16 & 0xff;  //第三位
     d=step_count >>24 & 0xff;  //第四位,最低位
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<4; i++ ) //填充缓冲 ,
	      {
	      I2c_Buf_Read[i]=0;      // 清空接收缓冲区
	      I2c_Buf_Write[i]=0;
	      
	      }
	      I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     
      
	     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 5, 4); //0x05 ~0x09
		 HAL_Delay(200);
		 #if 0
		  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4); 
	   if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
		 {
		    if(I2c_Buf_Read[1] == I2c_Buf_Write[1]) //
				if(I2c_Buf_Read[2] == I2c_Buf_Write[2]) //
				if(I2c_Buf_Read[3] == I2c_Buf_Write[3]) //
					{
						LED1_ON;
						LED2_ON;
						printf("eeprom is save OK \n");
						//while(1);
					}
		 
		 }
		 else printf("eeprom is save error \n");
		 #endif 
	
     //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	
	     if(flag_w==1) //表示写入成功
	        {
	        LED1_ON;
			HAL_Delay(10);
			LED1_OFF;
		    }
		   else
         printf("eeprom check error \n");				 
		   	//while(1);
	}	 

}

/*******************************************
 *
 *函数名称：
 *函数功能：存储A2的数据，在A1 的EEPROM中。
 *
 *
 *
********************************************/
void A2_Data_Save_To_EEPROM(void)
{
    uint8_t flag,flag_w,i;
     uint32_t a,b,c;
   
	
	     c=I2C_RX_SAVE_Buffer[2];  //最低位
	     b=I2C_RX_SAVE_Buffer[1];
	     a=I2C_RX_SAVE_Buffer[0];  //最高位
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<3; i++ ) //填充缓冲 ,
	      {
	      I2c_Buf_Read[i]=0;      // 清空接收缓冲区
	      I2c_Buf_Write[i]=0;
	      
	      }
	          
	      I2c_Buf_Write[0]=a;
	      I2c_Buf_Write[1]=b;
	      I2c_Buf_Write[2]=c;
	     
      
	     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 100, 3); //0x05 ~0x09
		 HAL_Delay(200);
		 #if 0
		  EEPROM_ReadBytes(I2c_Buf_Read, 100, 3); 
	   if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
		 {
		    if(I2c_Buf_Read[1] == I2c_Buf_Write[1]) //
				if(I2c_Buf_Read[2] == I2c_Buf_Write[2]) //
				if(I2c_Buf_Read[3] == I2c_Buf_Write[3]) //
					{
						LED1_ON;
						LED2_ON;
						printf("eeprom is save OK \n");
						//while(1);
					}
		 
		 }
		 else printf("eeprom is save error \n");
		 #endif 
	
     //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	
	     if(flag_w==1) //表示写入成功
	        {
	        LED1_ON;
			HAL_Delay(10);
			LED1_OFF;
		    }
		   else
         printf("eeprom check error \n");				 
		   	//while(1);
	}	 



}

/**********************************************************
 *
 *函数名：
 *函数功能：A1读取A2马达实时参数值，从A1的EEPROM中读取
 *
 *
***********************************************************/
void A1_Read_A2_Data_EEPROM(void)
{

       int32_t temp1,temp2,temp3;
	   uint8_t flag,i;
	   uint32_t  A2_realtime;
	   uint8_t sendbuffer[6]={0xa1,0x03,00,00,00,0x0b};
	   
      
         flag=EEPROM_CheckOk();
	     if(flag==1)
		 {
    	  for(i=0;i<4;i++)
		  {
		    I2c_Buf_Read[i]=0;
		  }
		  /*??EEPROM??????????????I2c_Buf_Read?? */ 
       	  EEPROM_ReadBytes(I2c_Buf_Read, 100, 3);  //马达走的位置的数据，存储位置
		  HAL_Delay(200);

          sendbuffer[4]=I2c_Buf_Read[2];
	      sendbuffer[3]=I2c_Buf_Read[1];
	      sendbuffer[2]=I2c_Buf_Read[0];
		  
		// temp1=Hex2oct_MSB(I2c_Buf_Read[1]);  
	     temp1=Hex2oct_MD1(I2c_Buf_Read[0]);
	     temp2=Hex2oct_MD2(I2c_Buf_Read[1]);
	     temp3=Hex2oct_LSB(I2c_Buf_Read[2]);
		 A2_realtime=temp1+temp2+temp3;
		 
		  
      
	     printf("A2 real position= %d \n", A2_realtime);
         HAL_UART_Transmit(&husartx,sendbuffer,6,12);		
       
         }
        
       else 
       {
          printf("A1 read A2 data is ERROR !!!\n");
       } 
}
