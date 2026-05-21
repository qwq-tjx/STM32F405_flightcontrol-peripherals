#include "stm32f4xx.h"


void My_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // ????
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    // GPIO???? - PC6??TIM3_CH1
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    
    // GPIO??
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // ???????
    // ??????? = 84MHz
    // PWM?? = 84MHz / (PSC+1) / (ARR+1) = 1kHz
    // ?? PSC=83, ARR=999, ?? 84MHz/84/1000 = 1000Hz = 1kHz
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;        // ARR = 999
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;       // PSC = 83
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // PWM????
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;                 // ?????10% (100/1000)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    
    // ????????
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    // ???????
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    // ??TIM3
    TIM_Cmd(TIM3, ENABLE);
}

// ??PWM???
void PWM_SetDuty(uint8_t duty_percent)
{
    uint32_t pulse_value;
    
    // ??????? 0-100%
    if (duty_percent > 100) duty_percent = 100;
    
    // ???????
    // ??????1000,??????????? = 1000 * (duty_percent / 100)
    pulse_value = (1000 * duty_percent) / 100;
    
    // ?????
    TIM_SetCompare1(TIM3, pulse_value);
}
