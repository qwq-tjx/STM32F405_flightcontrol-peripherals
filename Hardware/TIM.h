#ifndef __TIM_H
#define __TIM_H
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx, u32 chx, u32 par, u32 mar, u16 ndtr, uint32_t priority);

void TIM8_PWM_Init(u32 arr,u32 psc);

// 任务调度定时器初始化
void TIM6_Init(u32 arr, u32 psc);  // 1秒周期定时器（电池检测）
void TIM7_Init(u32 arr, u32 psc);  // 10ms周期定时器（MAVLink发送）

#endif
