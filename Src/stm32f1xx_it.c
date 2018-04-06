
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "usart/bsp_usartx.h"
#include "key/bsp_key.h"
#include "led/bsp_led.h"
#include "DRV8825/drv8825.h"
#include "StepMotor/bsp_STEPMOTOR.h" 
#include "spi/bsp_spi.h"
#include "i2c_slave/bsp_I2C.h"
#define STEPMOTOR_MICRO_STEP      32  // ²½½øµç»úÇý¶¯Æ÷Ï¸·Ö£¬±ØÐëÓëÇý¶¯Æ÷Êµ¼ÊÉèÖÃ¶ÔÓ¦

extern __IO uint32_t pulse_count; /*  Âö³å¼ÆÊý£¬Ò»¸öÍêÕûµÄÂö³å»áÔö¼Ó2 */
extern __IO uint16_t Origin_Pos;
extern __IO uint16_t Absolute_Pos   ;
extern __IO uint8_t flag_abs;   //Ç°½øºóÍËµÄ±êÖ¾Î»¡
extern __IO uint8_t Abs_high_byte;
extern __IO uint8_t Abs_low_byte;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USART1 global interrupt.
*/
void USARTx_IRQHANDLER(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&husartx);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}


/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/
/*È¦ÊýÈ·¶¨Öµ*/
/* USER CODE BEGIN 1 */
#if 0
void STEPMOTOR_TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htimx_STEPMOTOR);
}
#endif 
/**
* @brief This function handles EXTI line0 interrupt.
*/
#if 0
void KEY3_EXTI_IRQHandler(void)  //wt.edit KEY1 modify KEY3
{


  /* USER CODE END EXTI0_IRQn 0 */
	
 HAL_GPIO_EXTI_IRQHandler(KEY3_GPIO_PIN);
 
              DRV8825_Stop_Move();
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
		__HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
	  


  /* USER CODE END EXTI0_IRQn 1 */
}
#endif
#if 0
/***********************************************************************
*
* @brief This function handles EXTI line[15:10] interrupts.
*Ô­µãÉèÖÃ
***********************************************************************/
void KEY1_EXTI_IRQHandler(void)
{
/* USER CODE END EXTI15_10_IRQn 0 */
  //Origin_Pos=Read_Origin_Position();
  HAL_GPIO_EXTI_IRQHandler(KEY1_GPIO_PIN);
  	     
	      Toggle_Pulse=300;     
        LED1_ON;
			LED2_ON;
			HAL_Delay(100);	
			LED1_OFF;
			LED2_OFF;
			HAL_Delay(100);
			LED1_ON;
			LED2_ON;
		    HAL_Delay(100);
           pulse_count=0;
	        DRV8825_SLEEP_DISABLE();
	        DRV8825_OUTPUT_ENABLE();
          DRV8825_CCW_NormalMove();
	     HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
			
	 if(pulse_count==STEPMOTOR_MICRO_STEP*200*1)  // ×ª¶¯10È¦ºóÍ£»ú 
    {
      Toggle_Pulse=300;     
			/* Æô¶¯±È½ÏÊä³ö²¢Ê¹ÄÜÖÐ¶Ï */
      HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
      DRV8825_OUTPUT_DISABLE(); // Í£»ú
	  //DRV8825_SLEEP_DISABLE();
    }
	//__HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
/* USER CODE END EXTI15_10_IRQn 1 */
}

void KEY2_EXTI_IRQHandler(void)
{


  /* USER CODE END EXTI15_10_IRQn 0 */
	//Origin_Pos=Read_Origin_Position();
    HAL_GPIO_EXTI_IRQHandler(KEY2_GPIO_PIN);
//	Absolute_Pos=DRV8825_CCCW_AbsPosMove(Abs_high_byte,Abs_low_byte)-Origin_Pos;
	        pulse_count=0; 
			DRV8825_CW_NormalMove();
	      //  HAL_Delay(Absolute_Pos*295);
			DRV8825_SLEEP_DISABLE();
	if(pulse_count==STEPMOTOR_MICRO_STEP*200*2*Origin_Pos)  // ×ª¶¯10È¦ºóÍ£»ú 
    {
      /* Æô¶¯±È½ÏÊä³ö²¢Ê¹ÄÜÖÐ¶Ï */
     // HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
      DRV8825_OUTPUT_ENABLE(); // Í£»ú
	   DRV8825_SLEEP_DISABLE();
			
    }
	}
	
#endif

  /* USER CODE END EXTI15_10_IRQn 1 */

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/
/* USER CODE BEGIN 1 */
#if 1
void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi_SPI);
}
#endif

/* USER CODE BEGIN 1 */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&I2cHandle);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
* @brief This function handles I2C1 error interrupt.
*/
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&I2cHandle);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
