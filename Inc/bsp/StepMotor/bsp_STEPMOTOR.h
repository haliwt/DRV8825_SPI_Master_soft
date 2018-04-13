#ifndef __STEPMOTOR_TIM_H__
#define __STEPMOTOR_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  run_state ;  // 电机旋转状态
  __IO uint8_t  dir ;        // 电机旋转方向
  __IO int32_t  min_delay;   // 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t  rel_step;		 //	相对运动的步数
}speedRampData;

/* 宏定义 --------------------------------------------------------------------*/
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
#define STEPMOTOR_TIM_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()     // 输出控制脉冲给电机驱动器
#define STEPMOTOR_TIM_PUL_PORT                GPIOA                            // 对应驱动器的PUL-（驱动器使用共阳接法）
#define STEPMOTOR_TIM_PUL_PIN                 GPIO_PIN_8                       // 而PLU+直接接开发板的5V(或者3.3V)

/**DRV8825 方向控制*/
#define STEPMOTOR_DIR_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_DIR_PORT                    GPIOA                            // 对应驱动器的DIR-（驱动器使用共阳接法）
#define STEPMOTOR_DIR_PIN                     GPIO_PIN_3                      // 而DIR+直接接开发板的5V(或者3.3V)
#define STEPMOTOR_DIR_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_RESET)

/*DRV8825  片选信号*/
#define STEPMOTOR_ENA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()     // 电机脱机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_ENA_PORT                    GPIOB                            // 对应驱动器的ENA-（驱动器使用共阳接法）
#define STEPMOTOR_ENA_PIN                     GPIO_PIN_13                      // 而ENA+直接接开发板的5V(或者3.3V)

#define DRV8825_DIR_CW()                      HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_SET)
#define DRV8825_DIR_CCW()                     HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_RESET)

#define DRV8825_OUTPUT_ENABLE()               HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT,STEPMOTOR_ENA_PIN,GPIO_PIN_SET)
#define DRV8825_OUTPUT_DISABLE()              HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT,STEPMOTOR_ENA_PIN,GPIO_PIN_RESET)


/*DRV8825 SLEEP 控制*/
#define STEPMOTOR_SLEEP_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()     // ??????,????????????
#define STEPMOTOR_SLEEP_PORT                    GPIOB                            // ??STEPMOTOR?ENA+(?????????)
#define STEPMOTOR_SLEEP_PIN                     GPIO_PIN_14                      // ?ENA-???????GND  wt.edit 2017.11.01
#define STEPMOTOR_SLEEP_ENABLE()                STEPMOTOR_SLEEP_PORT->BSRR = (uint32_t)STEPMOTOR_SLEEP_PIN << 16  // =1;
#define STEPMOTOR_SLEEP_DISABLE()               STEPMOTOR_SLEEP_PORT->BSRR = STEPMOTOR_SLEEP_PIN                  // 0 ,??????????
#define DRV8825_SLEEP_DISABLE()                 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)    //高电平工作，没有光耦和。
#define DRV8825_SLEEP_ENABLE()                  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)    // 低电平进入睡眠模式。



// 定义定时器预分频，定时器实际时钟频率为：72MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define STEPMOTOR_TIM_PRESCALER               3  // 步进电机驱动器细分设置为：   32  细分
//#define STEPMOTOR_TIM_PRESCALER               7  // 步进电机驱动器细分设置为：   16  细分
//#define STEPMOTOR_TIM_PRESCALER               15  // 步进电机驱动器细分设置为：   8  细分
//#define STEPMOTOR_TIM_PRESCALER               31  // 步进电机驱动器细分设置为：   4  细分
//#define STEPMOTOR_TIM_PRESCALER               63  // 步进电机驱动器细分设置为：   2  细分
//#define STEPMOTOR_TIM_PRESCALER               127  // 步进电机驱动器细分设置为：   1  细分


// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// 定义高级定时器重复计数寄存器值
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0


#define FALSE                                 (uint8_t)0
#define TRUE                                  (uint8_t)1
#define CW                                    0 // 顺时针
#define CCW                                   1 // 逆时针

#define STOP                                  0 // 加减速曲线状态：停止
#define RUN                                   1 // 加减速曲线状态：匀速阶段
#define T1_FREQ                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER+1)) // 频率ft值
#define FSPR                                  200//步进电机单圈步数  步距角:1.8° 360/1.8 = 200 需要200步转一圈
#define MICRO_STEP                            32 // 步进电机驱动器细分数
#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;
extern __IO uint16_t Toggle_Pulse;
/* 函数声明 ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_Init(void);

void STEPMOTOR_TIMx_Init(void);
void STEPMOTOR_AxisMoveRel(int32_t step, uint32_t speed);
void STEPMOTOR_AxisMoveAbs(int32_t targert_step, uint32_t speed);
void STEPMOTOR_PC_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed);



#endif	/* __STEPMOTOR_TIM_H__ */
/******************* *****END OF FILE****/
