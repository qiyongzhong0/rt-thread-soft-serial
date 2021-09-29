/*
 * soft_serial.h
 *
 * Change Logs:
 * Date           Author        Notes
 * 2021-09-29     qiyongzhong   create v1.0
 */

#ifndef __SOFT_SERIAL_H__
#define __SOFT_SERIAL_H__

#include <soft_serial_stm32f103rc.h>
#include <soft_serial_stm32f103ze.h>

#ifndef SS_TIM_CLK_FREQ
#define SS_TIM_CLK_FREQ     8000000
#endif

#ifndef SS_TIM_PCLK1_MUL
#define SS_TIM_PCLK1_MUL    2 //APB1 timer clock multiplying power
#endif

#ifndef SS_TIM_PCLK2_MUL
#define SS_TIM_PCLK2_MUL    1 //APB1 timer clock multiplying power
#endif

#ifndef SS_CHK_RX_BEGIN_BIT
#define SS_CHK_RX_BEGIN_BIT //using recv begin bit check, not recommended when baudrate is high 
#endif

#ifndef SS_CHK_RX_PARITY
//#define SS_CHK_RX_PARITY  //using recv parity bit check, not recommended
#endif

int ss_scoms_init(void);//initialize and register soft serial, it is auto initialized

#endif

