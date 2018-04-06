#ifndef __BSP_I2C_H__
#define	__BSP_I2C_H__
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define I2C_OWN_ADDRESS_STM32                            0x3E              // stm32����I2C��ַ
#define I2C_SPEEDCLOCK                             400000            // I2Cͨ������(���Ϊ400K)
#define I2C_DUTYCYCLE                              I2C_DUTYCYCLE_2   // I2Cռ�ձ�ģʽ��1/2 

#define I2Cx                                I2C1
#define I2C_RCC_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define I2C_RCC_CLK_DISABLE()               __HAL_RCC_I2C1_CLK_DISABLE()

#define I2C_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()   
#define I2C_GPIO_PORT                       GPIOB   
#define I2C_SCL_PIN                         GPIO_PIN_6
#define I2C_SDA_PIN                         GPIO_PIN_7

/* 
 * EEPROM 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */
/* EEPROM Addresses defines */ 
#define I2C_ADDRESS                         0x30F

/* ��չ���� ------------------------------------------------------------------*/
extern I2C_HandleTypeDef I2cHandle;

/* �������� ------------------------------------------------------------------*/
void               MX_I2C_EEPROM_Init(void);
HAL_StatusTypeDef  I2C_EEPROM_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
void I2C_SLAVE_RX_FUN(void);

#endif /* __BSP_I2C_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
