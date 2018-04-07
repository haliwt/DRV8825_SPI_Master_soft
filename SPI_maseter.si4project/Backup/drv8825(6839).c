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
extern uint8_t repcdata[3]; //�ϴ�����λ��������
extern __IO uint16_t Toggle_Pulse ;
extern __IO uint32_t pulse_count; /*  ���������һ�����������������2 */
extern uint8_t I2c_Buf_Write[256];
extern uint8_t I2c_Buf_Read[256];

extern speedRampData srd  ;
static __IO int32_t  step_position=0 ;
__IO uint8_t stop_flag=0;
extern __IO uint32_t PulseNumbers;
extern __IO uint32_t step_count; 
__IO uint8_t home_flag=0; 
extern __IO uint32_t step_count;  //��PC�����ʹ洢������������
__IO uint8_t NewOrigin_flag=0;
extern __IO uint8_t back_flag;
extern __IO int32_t  LimPosi ; //�������ޱ�־λ  True:���Ｋ��λ  False:δ���Ｋ��λ
extern __IO int32_t  LimNega ; //�������ޱ�־λ
extern __IO uint8_t END_STOP_FLAG;  //�������е��յ㣬ֹͣ��־λ

/***************************************
*
*��������:�����ٶ�����
*�������趨�����ٶ�
*DRV8825 nENBL --- Hight level is diable ,
               ----Low leve is enable
****************************************/
void DRV8825_SetSpeed(uint8_t high_b,uint8_t low_b)
{
	  uint8_t i,flag;
	  flag=EEPROM_CheckOk();
	   if(flag==1)
	   {
     // for ( i=0; i<2; i++ ) //��仺�� ,
      {
      I2c_Buf_Read[0]=0;      // ��ս��ջ�����
      I2c_Buf_Read[1]=0; 
      I2c_Buf_Write[0] = high_b;   // Ϊ���ͻ������������
	  I2c_Buf_Write[1] = low_b;
      }
     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
	 EEPROM_WriteBytes(I2c_Buf_Write, 16, 2);  
     HAL_Delay(100);
	
     //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
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
    if(i==2) //��ʾд��ɹ�
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
*�������ܣ���ȡ�ٶ�ֵ
*��������
*����ֵ����ȡ�����ٶ�ֵ ,���������,[2][3]
*
********************************************/
uint16_t DRV8825_ReadSpeed(void)
{
    uint8_t flag;
	  uint16_t temp;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
       	 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //��ַ�� 0x0F,�ֽ�����2
		 temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
		
     	 }
		return temp;
    
}

/****************************************
*
*�������ܣ����λ���ƶ����Ե�ǰ��λ�ü��㡣
*����: �����16���ƣ�2���ֽ�?---����������
*����ֵ���ޣ���������λ�÷����ƶ�
*
***************************************/
void DRV8825_CCW_AxisMoveRel(uint8_t ccir_high,uint8_t ccir_mid,uint8_t ccir_low,uint16_t speed)
{
   uint32_t all_cir;
	 all_cir=ccir_high<<16|ccir_mid<<8|ccir_low;
	// STEPMOTOR_AxisMoveRel(1*all_cir*SPR,speed); 
	 STEPMOTOR_AxisMoveRel(1*all_cir,speed);   

}

/****************************************
*
*�������ܣ����λ���ƶ����Ե�ǰ��λ�ü��㡣
*����: �����16���ƣ�3���ֽڣ�������������
*����ֵ����,������λ�÷����ƶ�
*
***************************************/
void DRV8825_CW_AxisMoveRel(uint8_t cir_high,uint8_t cir_mid,uint8_t cir_low,uint16_t speed)
{
     int32_t all_cir;
	 all_cir=cir_high<<16|cir_mid<<8|cir_low;
	 //STEPMOTOR_AxisMoveRel(-1*all_cir*SPR,speed); 
	 STEPMOTOR_AxisMoveRel(-1*all_cir,speed); 

}
/**********************************
*
*�������ܣ�����ֹͣ
*
**********************************/
void DRV8825_StopMove(void)
{
       
        stop_flag=1;
	   // PulseNumbers=0;  //
        HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
       // �ر�ͨ��
       TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);        
       __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
      // DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_SLEEP_ENABLE(); //����ʡ��ģ?
       DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_OUTPUT_DISABLE();    //DRV8825оƬ�ߵ�ƽ��û�����?
       Display_EEPROM_Value();

}
/************************************
*
*����?  : Read_Origin_Position()?
*���ܣ���ȡԭ���λ��
*��������
*����ֵ��ԭ���λ�����ꡣ
*
************************************/
uint16_t Read_Origin_Position(void)
{
     uint8_t flag;
	   uint16_t origin_pos;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
	 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
   	 EEPROM_ReadBytes(I2c_Buf_Read, 0, 2); 
		 HAL_Delay(200);
     origin_pos=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
		 return origin_pos;
    
	  }
		else return 0;
		
}

/******************************************************
*
*�ײ���㺯��
* DRV8825_Pulse_AxisMvoeRel(int32_t pulse ,int32_t speed)
*�������ܣ�������ģʽ����,���λ��
*    �����λ�ã�ת��һȦ ��200x32=6400�����塣
*������pulse , ��������speed �ٶȡ�
*����ֵ����
*
*******************************************************/
void DRV8825_Back_AxisMoveRel(int32_t backstep, uint32_t backspeed)
{
   uint32_t tim_count;//��ȡ��ʱ���ļ���ֵ
   uint16_t i=10000;
  if(backstep < 0)      // ����Ϊ����
  {
	      
		srd.dir = CCW; // ��ʱ�뷽����ת,�˲��������﷽���ƶ���--����ߣ��˲���
	    STEPMOTOR_DIR_REVERSAL();
		//LimNega=step; //wt.edit 2018.01.16
		#if 1//��� buf ���ԭ��ģ�λ�ô���
		if((stop_flag==1) && (NewOrigin_flag==0))  //��stop����ִ�д����
		{
		  stop_flag=10;  //ִ�ж�ʱ���жϺ���
		 // back_flag=0;
		  backstep =-backstep; 
		  LimNega=backstep;
		 // temp_value=-step+DRV8825_Read_CurrentPosition();
		  step_position=DRV8825_Read_CurrentPosition();
		 // step_count=temp_value;
		  srd.rel_step  =backstep;
		  //PulseNumbers=DRV8825_Read_CurrentPosition();
		 // PulseNumbers= backstep- DRV8825_Read_CurrentPosition(); //wt.edit 18.03.11
		    PulseNumbers=step_position;
		}
		
		else 
		#endif	
		{
	    backstep =-backstep;   // ��ȡ��������ֵ
		step_position=backstep;//wt.edit 2018.03.11
		LimNega=backstep; //wt.edit 2018.01.16
		
		}
		
  }
  else
  {     
	    srd.dir = CW;  // ˳ʱ�뷽����ת---��ǰ�ߣ��������﷽���ƶ���
	    STEPMOTOR_DIR_FORWARD();
		LimPosi=backstep; 
		#if 1            //stop ����ͣ��
		if((stop_flag==1) && (NewOrigin_flag==0)) 
		{
			stop_flag=10;
			LimPosi=backstep;
			//temp_value=step-DRV8825_Read_CurrentPosition();
			//step_position=DRV8825_Read_CurrentPosition();
			PulseNumbers=DRV8825_Read_CurrentPosition();
			
			srd.rel_step  = backstep;
			//Display_EEPROM_Value();
		}
		else
		#endif 
	    {
			LimPosi=backstep;    //wt.edit 2018.01.16 
			srd.rel_step  = backstep;
		}
    	
  }
	
 // srd.rel_step  = step;	//��¼����˶��Ĳ���
  

  if(backstep != 0)    // ���Ŀ���˶�������Ϊ0
  {
    
    
    #if 1	
			
	do
		{
           // Toggle_Pulse=1200;
			srd.min_delay=1200;
		    srd.run_state = RUN;

			tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
			//�Ƚ�ֵ��CCRx=tim_count+srd.min_delay
		     __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay); // ���ö�ʱ���Ƚ�ֵ
			HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
			TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ�� 
		    DRV8825_OUTPUT_ENABLE();
    	}
	while(i--);
	#endif
	Toggle_Pulse=DRV8825_ReadSpeed();
	srd.min_delay=Toggle_Pulse;
    srd.run_state = RUN;
	
  }
  tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
	//�Ƚ�ֵ��CCRx=tim_count+srd.min_delay
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay); // ���ö�ʱ���Ƚ�ֵ
	HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
	TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ�� 
  DRV8825_OUTPUT_ENABLE();

}

/****************************************************************
*
*�������ܣ��洢���ﵱǰ��λ��,step_position
*��������
*����ֵ����
*
****************************************************************/
void DRV8825_Save_CurrentPosition(void)
{
     uint8_t flag,flag_w,i;
     uint32_t a,b,c,d;
    // a=PulseNumbers & 0xff;        //���λ
    // b=PulseNumbers >>8 & 0xff;   //�ڶ�λ
    // c=PulseNumbers >>16 & 0xff;  //����λ
    // d=PulseNumbers >>24 & 0xff;  //����λ,���λ
	   a=step_count & 0xff;        //���λ
     b=step_count >>8 & 0xff;   //�ڶ�λ
     c=step_count >>16 & 0xff;  //����λ
     d=step_count >>24 & 0xff;  //����λ,���λ
	
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<4; i++ ) //��仺�� ,
	      {
	      I2c_Buf_Read[i]=0;      // ��ս��ջ�����
	      I2c_Buf_Write[i]=0;
	      
	      }
	      I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     
      
	     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
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
	
     //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
   	
	     if(flag_w==1) //��ʾд��ɹ�
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
*�������ܣ���ȡ���ﵱǰλ�����ﵱǰ��λ��,step_position
*��������
*����ֵ���� I2c_Buf_Read[0]=MSB    I2c_Buf_Read[3]=LSB
*
****************************************************************/
uint32_t DRV8825_Read_CurrentPosition(void)
{
       uint8_t flag;
	   uint32_t result,temp1,temp2,temp3,temp4;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
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
 * �������ܣ���ʾһ����ǰ����λ�õĲ��������� 
 * �����������
 * ���ز����ޣ�
 *
***********************************************************/
void Display_EEPROM_Value(void)
{
       uint32_t temp1,temp2,temp3,temp4;
	   uint8_t flag,i;
	   uint8_t sendbuffer[6]={0xa1,0x03,00,00,00,0x0b};
	   uint32_t value,real_value;
	   
	   real_value=step_count;
      // send_realn=Decimal_TO_Hex(real_value);
	
	  
	   sendbuffer[4]=real_value & 0xff;
	   sendbuffer[3]=real_value>>8 & 0xff;
	   sendbuffer[2]=real_value>>16 & 0xff; 
      // HAL_UART_Transmit(&husartx,sendbuffer,6,20);
	   
	    flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	  for(i=0;i<4;i++)
		  {
		    I2c_Buf_Read[i]=0;
		  }
		  /*??EEPROM??????????????I2c_Buf_Read?? */ 
       	 EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //�����ߵ�λ�õ����ݣ��洢λ��
		  HAL_Delay(200);
		 
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
	     temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
	     temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
	     temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
		 value=temp1+temp2+temp3+temp4;
		 
		  
		// temp=(I2c_Buf_Read[3]<<24)|(I2c_Buf_Read[2]<<16)|(I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0]);
        // HAL_UART_Transmit(&husartx,I2c_Buf_Read,5,4);
		// printf("--------------------------- \n");
		// printf("temp current position= %ld \n",temp);
		 printf("EEPROM position= %ld \n",value);
		 printf("real position= %ld \n",real_value);
         HAL_UART_Transmit(&husartx,sendbuffer,6,12);		  
		 
		 }
}

/*********************************************************
 *
 * �������ƣ�A2_Pulse_RealTime_Value
 * �������ܣ�������λ���ڶ�����ʵʱ��������
 * ����������� --�����ǲ����ϴ�
 * ���ز����ޣ�
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
 * �������ƣ�A2_Pulse_EEPROM_Value
 * �������ܣ�������λ���ڶ���EEPROM of value
 * �����������
 * ���ز����ޣ�
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
*�������ܣ����EEPROM�Ĵ洢����ֵλ0
*
************************************************/
void EEPROM_Clear_Buf(void)
{
 uint16_t i;
  uint8_t flag,flag_w;
  flag=EEPROM_CheckOk();
  
  if(flag==1)
  {
    for ( i=0; i<10; i++ ) //??��?? ,
      {
      I2c_Buf_Read[i]=0;      // ???????????
      I2c_Buf_Write[i]=0;      
      }
     //??I2c_Buf_Write??????????????��??EERPOM?? 
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
 *��������:�洢��ǰ�����λ��,��EEPROM�еĴ洢λ����:0~3-4���ֽ�
 *�����������
 *����ֵ����
 *
****************************************************************/
void MOTOR_Save_NewHomePosition(void)
{
     uint8_t flag,flag_w,i;
     uint8_t a,b,c,d;
     a=PulseNumbers & 0xff;        //???��
     b=PulseNumbers >>8 & 0xff;   //???��
     c=PulseNumbers >>16 & 0xff;  //????��
     d=PulseNumbers >>24 & 0xff;  //????��,???��
	
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
          for ( i=0; i<4; i++ ) //���I2C_Buf_Read,and write
	      {
		      I2c_Buf_Read[i]=0;      // ???????????
		      I2c_Buf_Write[i]=0;      
	      }
		  I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     //??I2c_Buf_Write??????????????��??EERPOM?? 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 4); //λ���� 0x00��0x03���ĸ��ֽ�
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
		if(flag_w==1) //???��????
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
 *��������:�洢��������ԭ��ı�־λ��home_flag=1,
 *         home_flag==1,��ʾ�����������µ�ԭ��,������EEPROM
           �еĵ�ַ 0x04
 *�����������
 *����ֵ��1---��ʾ�������������µ�ԭ�����ꡣ
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
      
    
	 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 4, 1); //��ַ��0x04,д��һ������
	 HAL_Delay(100);
	 #if 0  //����д�������
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
   	
     if(flag_w==1) //???��????
        {
        LED1_TOGGLE ;
		return 1;
    	}
	 else 
	 	return 0;
}	 


/******************************************************
 *
 *�������ܣ��Ƕ�ȡ������ԭ��ı�־λ��home_flag
 *��������ޣ�
 *���ز����� 1 ---��ʾ����������ԭ��ɹ��� 0----��ʾû����������ԭ��-ʧ�ܡ�
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
		  EEPROM_ReadBytes(I2c_Buf_Read, 4, 1); //�ӵ�ַ��0x04,��һ���ֽڡ� 
		  HAL_Delay(200);  
          result=I2c_Buf_Read[0];
		  LED2_TOGGLE ;
		 
     	}
		 return result;
}
/***************************************************
 *
 *�������ܣ������µ�����ԭ��
 *
 *
*******************************************************/
void Set_NewOrigin_Position(void)
{
    uint8_t flag,flag_w,i;
   // uint32_t a,b,c,d;
	NewOrigin_flag=1;
    PulseNumbers=0;
    step_count=0;
	step_position=0;  //wt.edit 18.03.07
    flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<10; i++ ) //清楚 EEPROM 存储的buffer清零
	      {
	      I2c_Buf_Read[i]=0;      // ��ս��ջ�����
	      I2c_Buf_Write[i]=0;
	      }
	     
     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 10); //0x05 ~0x09
		 HAL_Delay(50);
	   //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
   	 DRV8825_Save_CurrentPosition();
	  }	 
		 if(flag_w==1) //��ʾд��ɹ�
	    {
	    LED1_ON;
			HAL_Delay(10);
			LED1_OFF;
		   }
		   else 
		   	while(1);


}

/*******************************************
*
*�������ܣ���λ����ȡ������ٶ�ֵ
*��������
*����ֵ����ȡ�����ٶ�ֵ ,���������,[2][3]
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
			 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��	
			 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //��ַ�� 0x0F,�ֽ�����2
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
*�������ܣ���ȡ�ڶ��������ٶ�ֵ A2
*��������
*����ֵ����ȡ�����ٶ�ֵ ,���������,[2][3]
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
*�������ܣ������ƶ��ľ���λ�����꺯�� ---��ʱ��
*������1��2��3 ��3���ֽ�����---������ 0x01
*���غ�������
*
*********************************************************************************************************************/
void STEPMOTOR_CCW_AxisMoveAbs( uint8_t ab_h,uint8_t ab_m,uint8_t ab_low, uint32_t speed) //����λ���ƶ�����-��ʱ��
{
   
	 int32_t ccw_targert_step;
	 
	ccw_targert_step=step_position -(ab_h<<16|ab_m<<8|ab_low);
    STEPMOTOR_AxisMoveRel(-1*ccw_targert_step ,speed); 
   

}
/********************************************************************************************************************
*
*�������ܣ������ƶ��ľ���λ�����꺯?--?--˳ʱ��
*������1��2��3 ��3���ֽ�����---������ 0x01
*���غ�������
*
*********************************************************************************************************************/
 void STEPMOTOR_CW_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed) //����λ���ƶ�����-��ʱ��
 {
       
         int32_t cw_targert_step;
	   
        cw_targert_step=step_position - (int32_t)(abs_high<<16|abs_mid<<8|abs_low);
        STEPMOTOR_AxisMoveRel(1*cw_targert_step,speed);
    
   }

/****************************************************************
 *
 *�������ƣ�
 *�������ܣ��洢���ﵽ�յ��λ�ã������㣬�Զ���ȡ
 *��������
 *����ֵ����
 *
****************************************************************/
void Motor_Save_EndPosition(void)
{
     uint8_t flag,flag_w,i;
     uint32_t a,b,c,d;
   
	
	 a=step_count & 0xff;        //���λ
     b=step_count >>8 & 0xff;   //�ڶ�λ
     c=step_count >>16 & 0xff;  //����λ
     d=step_count >>24 & 0xff;  //����λ,���λ
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<4; i++ ) //��仺�� ,
	      {
	      I2c_Buf_Read[i]=0;      // ��ս��ջ�����
	      I2c_Buf_Write[i]=0;
	      
	      }
	      I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     
      
	     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
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
	
     //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
   	
	     if(flag_w==1) //��ʾд��ɹ�
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