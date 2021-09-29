/*
 * soft_serial_stm32f103rc.h
 *
 * Change Logs:
 * Date           Author        Notes
 * 2021-09-29     qiyongzhong   create v1.0
 */

#ifndef __SOFT_SERIAL_STM32F103RC_H__
#define __SOFT_SERIAL_STM32F103RC_H__

#ifdef CHIP_NAME_STM32F103RC

//#define SS_MODE_HALF      //half-duplex mode, rx channel and tx channel may is same

#define SS_SCOM_CFG_TABLE \
{\
    {"scom1", {TIM3, LL_TIM_CHANNEL_CH1, GPIOA, GPIO_PIN_6}, {TIM3, LL_TIM_CHANNEL_CH2, GPIOA, GPIO_PIN_7}},\
    {"scom2", {TIM3, LL_TIM_CHANNEL_CH3, GPIOB, GPIO_PIN_0}, {TIM3, LL_TIM_CHANNEL_CH4, GPIOB, GPIO_PIN_1}},\
    {"scom3", {TIM8, LL_TIM_CHANNEL_CH1, GPIOC, GPIO_PIN_6}, {TIM8, LL_TIM_CHANNEL_CH2, GPIOC, GPIO_PIN_7}},\
    {"scom4", {TIM8, LL_TIM_CHANNEL_CH3, GPIOC, GPIO_PIN_8}, {TIM8, LL_TIM_CHANNEL_CH4, GPIOC, GPIO_PIN_9}},\
    {"scom5", {TIM4, LL_TIM_CHANNEL_CH1, GPIOB, GPIO_PIN_6}, {TIM4, LL_TIM_CHANNEL_CH2, GPIOB, GPIO_PIN_7}},\
    {"scom6", {TIM4, LL_TIM_CHANNEL_CH3, GPIOB, GPIO_PIN_8}, {TIM4, LL_TIM_CHANNEL_CH4, GPIOB, GPIO_PIN_9}} \
}

#define SS_TIM_CH_REMAP()  do{ \
    __HAL_AFIO_REMAP_TIM3_DISABLE();    \
    __HAL_AFIO_REMAP_TIM4_DISABLE();    \
}while(0)

//#define SS_USING_TIM1
//#define SS_USING_TIM2
#define SS_USING_TIM3
#define SS_USING_TIM4
//#define SS_USING_TIM5
#define SS_USING_TIM8

#endif
#endif

