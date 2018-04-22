#ifndef __STEPMOTOR_TIM_H__
#define __STEPMOTOR_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  run_state ;  // �����ת״̬
  __IO uint8_t  dir ;        // �����ת����
  __IO int32_t  min_delay;   // ��С��������(����ٶȣ������ٶ��ٶ�)
  __IO int32_t  rel_step;		 //	����˶��Ĳ���
}speedRampData;

/* �궨�� --------------------------------------------------------------------*/
#define STEPMOTOR_TIMx                        TIM1
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM1_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM1_CLK_DISABLE()
#define STEPMOTOR_TIMx_IRQn                   TIM1_CC_IRQn
#define STEPMOTOR_TIMx_IRQHandler             TIM1_CC_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_x                TIM_CHANNEL_1
#define STEPMOTOR_TIM_IT_CCx                  TIM_IT_CC1
#define STEPMOTOR_TIM_FLAG_CCx                TIM_FLAG_CC1
#define STEPMOTOR_TIMx_IRQn                   TIM1_CC_IRQn
#define STEPMOTOR_TIMx_IRQHandler             TIM1_CC_IRQHandler


#define STEPMOTOR_TIM_CHANNEL_x               TIM_CHANNEL_1
#define STEPMOTOR_TIM_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL_PORT                GPIOA                            // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL_PIN                 GPIO_PIN_8                       // ��PLU+ֱ�ӽӿ������5V(����3.3V)

/**DRV8825 �������*/
#define STEPMOTOR_DIR_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR_PORT                    GPIOA                            // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR_PIN                     GPIO_PIN_3                      // ��DIR+ֱ�ӽӿ������5V(����3.3V)
#define STEPMOTOR_DIR_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_RESET)

/*DRV8825  Ƭѡ�ź�*/
#define STEPMOTOR_ENA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()     // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA_PORT                    GPIOB                            // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA_PIN                     GPIO_PIN_13                      // ��ENA+ֱ�ӽӿ������5V(����3.3V)

#define DRV8825_DIR_CW()                      HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_SET)
#define DRV8825_DIR_CCW()                     HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_RESET)

#define DRV8825_OUTPUT_ENABLE()               HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT,STEPMOTOR_ENA_PIN,GPIO_PIN_SET)
#define DRV8825_OUTPUT_DISABLE()              HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT,STEPMOTOR_ENA_PIN,GPIO_PIN_RESET)


/*DRV8825 SLEEP ����*/
#define STEPMOTOR_SLEEP_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()     // ??????,????????????
#define STEPMOTOR_SLEEP_PORT                    GPIOB                            // ??STEPMOTOR?ENA+(?????????)
#define STEPMOTOR_SLEEP_PIN                     GPIO_PIN_14                      // ?ENA-???????GND  wt.edit 2017.11.01
#define STEPMOTOR_SLEEP_ENABLE()                STEPMOTOR_SLEEP_PORT->BSRR = (uint32_t)STEPMOTOR_SLEEP_PIN << 16  // =1;
#define STEPMOTOR_SLEEP_DISABLE()               STEPMOTOR_SLEEP_PORT->BSRR = STEPMOTOR_SLEEP_PIN                  // 0 ,??????????
#define DRV8825_SLEEP_DISABLE()                 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)    //�ߵ�ƽ������û�й���͡�
#define DRV8825_SLEEP_ENABLE()                  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)    // �͵�ƽ����˯��ģʽ��



// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��72MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define STEPMOTOR_TIM_PRESCALER               3  // �������������ϸ������Ϊ��   32  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               7  // �������������ϸ������Ϊ��   16  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               15  // �������������ϸ������Ϊ��   8  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               31  // �������������ϸ������Ϊ��   4  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               63  // �������������ϸ������Ϊ��   2  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               127  // �������������ϸ������Ϊ��   1  ϸ��


// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0


#define FALSE                                 (uint8_t)0
#define TRUE                                  (uint8_t)1
#define CW                                    0 // ˳ʱ��
#define CCW                                   1 // ��ʱ��

#define STOP                                  0 // �Ӽ�������״̬��ֹͣ
#define RUN                                   1 // �Ӽ�������״̬�����ٽ׶�
#define T1_FREQ                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER+1)) // Ƶ��ftֵ
#define FSPR                                  200//���������Ȧ����  �����:1.8�� 360/1.8 = 200 ��Ҫ200��תһȦ
#define MICRO_STEP                            32 // �������������ϸ����
#define SPR                                   (FSPR*MICRO_STEP)   // ��תһȦ��Ҫ��������

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;
extern __IO uint16_t Toggle_Pulse;
/* �������� ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_Init(void);

void STEPMOTOR_TIMx_Init(void);
void STEPMOTOR_AxisMoveRel(int32_t step, uint32_t speed);
void STEPMOTOR_AxisMoveAbs(int32_t targert_step, uint32_t speed);
void STEPMOTOR_PC_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed);



#endif	/* __STEPMOTOR_TIM_H__ */
/******************* *****END OF FILE****/
