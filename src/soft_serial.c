/*
 * soft_serial.c
 *
 * Change Logs:
 * Date           Author        Notes
 * 2021-09-29     qiyongzhong   create v1.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <board.h>
#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_ll_rcc.h>
#include <soft_serial.h>

#define SS_EVT_RX_DONE  (1<<0)
#define SS_EVT_TX_DONE  (1<<1)

typedef struct{
    TIM_TypeDef *tim;
    uint32_t channel;
    GPIO_TypeDef *port;
    uint32_t pin;
}ss_ch_cfg_t;

typedef struct{
    char *name;
    ss_ch_cfg_t rx;
    ss_ch_cfg_t tx;
}ss_config_t;

typedef struct{
    rt_serial_t parent;
    struct rt_event evt;
    const ss_config_t *cfg;
    uint16_t bit_clk;
    uint8_t  init           : 1;//initialize sign
    uint8_t  rx_startup     : 1;//rx startup sign
    uint8_t  rx_parity_err  : 1;//rx rarity error sign
    uint8_t  tx_parity      : 1;//tx parity bit value
    uint8_t  irq_type       : 1;//irq_type, 0-rx, 1-tx
    uint8_t  tx_busy        : 1;//send busy sign
    uint8_t  rx_valid       : 1;//recv valid sign
    uint8_t  rx_valid_byte;
    uint8_t  rx_step;
    uint8_t  rx_byte;
    uint8_t  tx_step;
    uint8_t  tx_byte;
}ss_scom_t;

static const ss_config_t ss_cfg_table[] = SS_SCOM_CFG_TABLE;
    
#define SS_SCOM_TOTAL  (sizeof(ss_cfg_table)/sizeof(ss_cfg_table[0])) 

static ss_scom_t ss_scoms[SS_SCOM_TOTAL] = {0};

static uint8_t ss_cal_parity(uint8_t ch)//奇数个1返回 1
{
    ch^=(ch>>4);
    ch^=(ch>>2);
    ch^=(ch>>1);
    ch&=0x01;
    return(ch);
}

static void ss_gpio_clk_enable(GPIO_TypeDef *gpio_port)
{
    switch((uint32_t)gpio_port)
    {
#ifdef GPIOA
    case (uint32_t)GPIOA:
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
#endif

#ifdef GPIOB    
    case (uint32_t)GPIOB:
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
#endif

#ifdef GPIOC    
    case (uint32_t)GPIOC:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
#endif 
    
#ifdef GPIOD
    case (uint32_t)GPIOD:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
#endif

#ifdef GPIOE
    case (uint32_t)GPIOE:
        __HAL_RCC_GPIOE_CLK_ENABLE();
        break;  
#endif

#ifdef GPIOF 
    case (uint32_t)GPIOF:
        __HAL_RCC_GPIOF_CLK_ENABLE();
        break; 
#endif

#ifdef GPIOG 
    case (uint32_t)GPIOG:
        __HAL_RCC_GPIOG_CLK_ENABLE();
        break;
#endif

#ifdef GPIOH
    case (uint32_t)GPIOH:
        __HAL_RCC_GPIOH_CLK_ENABLE();
        break; 
#endif

    default:
        break;
    }
}

static void ss_pin_init(const ss_config_t *cfg)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SS_TIM_CH_REMAP();
    ss_gpio_clk_enable(cfg->rx.port);
    ss_gpio_clk_enable(cfg->tx.port);
    
    GPIO_InitStruct.Pin = cfg->rx.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(cfg->rx.port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = cfg->tx.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(cfg->tx.port, &GPIO_InitStruct);
}

static uint32_t ss_rx_pin_read(const ss_config_t *cfg)
{
    return(cfg->rx.port->IDR & cfg->rx.pin);
}

static void ss_tx_pin_write(const ss_config_t *cfg, int level)
{
    if (level)
        cfg->tx.port->ODR |= cfg->tx.pin;
    else
        cfg->tx.port->ODR &= ~cfg->tx.pin;
}

static void ss_tim_clk_enable(TIM_TypeDef *tim)
{
    switch((uint32_t)tim)
    {
#ifdef TIM1
    case (uint32_t)TIM1:
        __HAL_RCC_TIM1_CLK_ENABLE();
        break;
#endif
    
#ifdef TIM2
    case (uint32_t)TIM2:
        __HAL_RCC_TIM2_CLK_ENABLE();
        break;
#endif
    
#ifdef TIM3
    case (uint32_t)TIM3:
        __HAL_RCC_TIM3_CLK_ENABLE();
        break;
#endif
    
#ifdef TIM4
    case (uint32_t)TIM4:
        __HAL_RCC_TIM4_CLK_ENABLE();
        break;
#endif
    
#ifdef TIM5
    case (uint32_t)TIM5:
        __HAL_RCC_TIM5_CLK_ENABLE();
        break;
#endif
    
#ifdef TIM8
    case (uint32_t)TIM8:
        __HAL_RCC_TIM8_CLK_ENABLE();
        break;
#endif

    default:
        break;
    }
}

static uint32_t ss_tim_clock_get(TIM_TypeDef *tim)
{
    LL_RCC_ClocksTypeDef RCC_Clocks = {0};
    
    LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
    switch((uint32_t)tim)
    {
#ifdef TIM1
    case (uint32_t)TIM1:
        return(RCC_Clocks.PCLK2_Frequency * SS_TIM_PCLK2_MUL);
#endif
    
#ifdef TIM2
    case (uint32_t)TIM2:
        return(RCC_Clocks.PCLK1_Frequency * SS_TIM_PCLK1_MUL);
#endif
    
#ifdef TIM3
    case (uint32_t)TIM3:
        return(RCC_Clocks.PCLK1_Frequency * SS_TIM_PCLK1_MUL);
#endif
    
#ifdef TIM4
    case (uint32_t)TIM4:
        return(RCC_Clocks.PCLK1_Frequency * SS_TIM_PCLK1_MUL);
#endif
    
#ifdef TIM5
    case (uint32_t)TIM5:
        return(RCC_Clocks.PCLK1_Frequency * SS_TIM_PCLK1_MUL);
#endif
    
#ifdef TIM8
    case (uint32_t)TIM8:
        return(RCC_Clocks.PCLK2_Frequency * SS_TIM_PCLK2_MUL);
#endif

    default:
        return(0);
    }
}

static int ss_tim_irq_get(TIM_TypeDef *tim)
{
    switch((uint32_t)tim)
    {
#ifdef TIM1
    case (uint32_t)TIM1:
        return(TIM1_CC_IRQn);
#endif
    
#ifdef TIM2
    case (uint32_t)TIM2:
        return(TIM2_IRQn);
#endif
    
#ifdef TIM3
    case (uint32_t)TIM3:
        return(TIM3_IRQn);
#endif
    
#ifdef TIM4
    case (uint32_t)TIM4:
        return(TIM4_IRQn);
#endif
    
#ifdef TIM5
    case (uint32_t)TIM5:
        return(TIM5_IRQn);
#endif
    
#ifdef TIM8
    case (uint32_t)TIM8:
        return(TIM8_CC_IRQn);
#endif

    default:
        return(0);
    }
}

static void ss_tim_irq_enable(TIM_TypeDef *tim)
{
    int irq = ss_tim_irq_get(tim);
    HAL_NVIC_SetPriority(irq, 0, 0);
    HAL_NVIC_EnableIRQ(irq);
}

static void ss_tim_init(TIM_TypeDef *tim)
{
    if ( ! LL_TIM_IsEnabledCounter(tim))//timer is no init
    {
        LL_TIM_InitTypeDef TIM_InitStruct = {0};
        uint32_t tim_clk = ss_tim_clock_get(tim);

        //the clock must not lower than SS_CLK_FREQ and it must is a multiple of SS_CLK_FREQ
        RT_ASSERT((tim_clk >= SS_TIM_CLK_FREQ) && (tim_clk % SS_TIM_CLK_FREQ == 0));
        
        ss_tim_clk_enable(tim);
        LL_TIM_DeInit(tim);
        
        TIM_InitStruct.Prescaler = tim_clk/SS_TIM_CLK_FREQ - 1;
        TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
        TIM_InitStruct.Autoreload = 0xFFFF;
        TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
        TIM_InitStruct.RepetitionCounter = 0;
        LL_TIM_Init(tim, &TIM_InitStruct);
        LL_TIM_EnableARRPreload(tim);
        LL_TIM_SetClockSource(tim, LL_TIM_CLOCKSOURCE_INTERNAL);
        LL_TIM_EnableCounter(tim);

        ss_tim_irq_enable(tim);
    }
}

static void ss_tim_cap_enable(const ss_ch_cfg_t *rx_cfg)
{
    LL_TIM_CC_DisableChannel(rx_cfg->tim, rx_cfg->channel);
    
    LL_TIM_IC_SetActiveInput(rx_cfg->tim, rx_cfg->channel, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(rx_cfg->tim, rx_cfg->channel, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(rx_cfg->tim, rx_cfg->channel, LL_TIM_IC_FILTER_FDIV1_N4);
    LL_TIM_IC_SetPolarity(rx_cfg->tim, rx_cfg->channel, LL_TIM_IC_POLARITY_FALLING);
        
    switch(rx_cfg->channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_ClearFlag_CC1(rx_cfg->tim);
        LL_TIM_EnableIT_CC1(rx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_ClearFlag_CC2(rx_cfg->tim);
        LL_TIM_EnableIT_CC2(rx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_ClearFlag_CC3(rx_cfg->tim);
        LL_TIM_EnableIT_CC3(rx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_ClearFlag_CC4(rx_cfg->tim);
        LL_TIM_EnableIT_CC4(rx_cfg->tim);
        break;
    default:
        break;
    }
    
    LL_TIM_CC_EnableChannel(rx_cfg->tim, rx_cfg->channel);
}

static void ss_tim_comp_enable(const ss_ch_cfg_t *rxtx_cfg, uint32_t comp_val)
{
    LL_TIM_CC_DisableChannel(rxtx_cfg->tim, rxtx_cfg->channel);
    
    LL_TIM_OC_SetMode(rxtx_cfg->tim, rxtx_cfg->channel, LL_TIM_OCMODE_FROZEN);
    LL_TIM_OC_DisablePreload(rxtx_cfg->tim, rxtx_cfg->channel);
    LL_TIM_OC_DisableFast(rxtx_cfg->tim, rxtx_cfg->channel);
    LL_TIM_OC_DisableClear(rxtx_cfg->tim, rxtx_cfg->channel);
    
    switch(rxtx_cfg->channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(rxtx_cfg->tim, comp_val);
        LL_TIM_ClearFlag_CC1(rxtx_cfg->tim);
        LL_TIM_EnableIT_CC1(rxtx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(rxtx_cfg->tim, comp_val);
        LL_TIM_ClearFlag_CC2(rxtx_cfg->tim);
        LL_TIM_EnableIT_CC2(rxtx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(rxtx_cfg->tim, comp_val);
        LL_TIM_ClearFlag_CC3(rxtx_cfg->tim);
        LL_TIM_EnableIT_CC3(rxtx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(rxtx_cfg->tim, comp_val);
        LL_TIM_ClearFlag_CC4(rxtx_cfg->tim);
        LL_TIM_EnableIT_CC4(rxtx_cfg->tim);
        break;
    default:
        break;
    }
    
    LL_TIM_CC_EnableChannel(rxtx_cfg->tim, rxtx_cfg->channel);
}

#ifndef SS_MODE_HALF
static void ss_tim_cc_disable(const ss_ch_cfg_t *rxtx_cfg)
{
    LL_TIM_CC_DisableChannel(rxtx_cfg->tim, rxtx_cfg->channel);
    
    switch(rxtx_cfg->channel)
    {
    case LL_TIM_CHANNEL_CH1:   
        LL_TIM_DisableIT_CC1(rxtx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH2:   
        LL_TIM_DisableIT_CC2(rxtx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH3:   
        LL_TIM_DisableIT_CC3(rxtx_cfg->tim);
        break;
    case LL_TIM_CHANNEL_CH4:   
        LL_TIM_DisableIT_CC4(rxtx_cfg->tim);
        break;
    default:
        break;
    }
}
#endif

static uint32_t ss_tim_get_cc_val(ss_ch_cfg_t *rxtx_cfg)
{
    switch(rxtx_cfg->channel)
    {
    case LL_TIM_CHANNEL_CH1:   
        return(LL_TIM_OC_GetCompareCH1(rxtx_cfg->tim));
    case LL_TIM_CHANNEL_CH2:   
        return(LL_TIM_OC_GetCompareCH2(rxtx_cfg->tim));
    case LL_TIM_CHANNEL_CH3:   
        return(LL_TIM_OC_GetCompareCH3(rxtx_cfg->tim));
    case LL_TIM_CHANNEL_CH4:   
        return(LL_TIM_OC_GetCompareCH4(rxtx_cfg->tim));
    default:
        return(0);
    }
}
static void ss_tim_set_cc_val(ss_ch_cfg_t *rxtx_cfg, uint32_t cc_val)
{
    switch(rxtx_cfg->channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(rxtx_cfg->tim, cc_val);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(rxtx_cfg->tim, cc_val);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(rxtx_cfg->tim, cc_val);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(rxtx_cfg->tim, cc_val);
        break;
    default:
        break;
    }
}

static uint32_t ss_tim_get_counter_val(TIM_TypeDef *tim)
{
    return(LL_TIM_GetCounter(tim));
}

static void ss_scom_startup_recv(ss_scom_t *scom)
{
    scom->rx_step = 0;
    ss_tim_cap_enable(&(scom->cfg->rx));
}

static void ss_scom_send_byte(ss_scom_t *scom, uint8_t byte)
{
    rt_uint32_t recved = 0;

    rt_event_control(&(scom->evt), RT_IPC_CMD_RESET, RT_NULL);
    
#ifdef SS_MODE_HALF
    if (    (scom->cfg->rx.tim == scom->cfg->tx.tim) //timer same
        &&  (scom->cfg->rx.channel == scom->cfg->tx.channel)//channel same
        &&  (scom->rx_step != 0))//recving
        
    {
        rt_event_recv(&(scom->evt), SS_EVT_RX_DONE, (RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR), rt_tick_from_millisecond(100), &recved);
    }
#endif

    scom->tx_step = 0;
    scom->tx_busy = 1;
    scom->tx_byte = byte;
    scom->tx_parity = ss_cal_parity(byte);
    ss_tx_pin_write(scom->cfg, 0);//send begin bit
    ss_tim_comp_enable(&(scom->cfg->tx), ss_tim_get_counter_val(scom->cfg->tx.tim) + scom->bit_clk);
    
    rt_event_recv(&(scom->evt), SS_EVT_TX_DONE, (RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR), rt_tick_from_millisecond(100), &recved);
}

static ss_scom_t * ss_tim_scom_get(TIM_TypeDef *tim, uint32_t channel)
{
    for(int i=0; i<SS_SCOM_TOTAL; i++)
    {
        ss_scom_t *scom = ss_scoms + i;
        if ((scom->cfg->rx.tim == tim) && (scom->cfg->rx.channel == channel))
        {
#ifdef SS_MODE_HALF
            if (    (scom->cfg->rx.tim == scom->cfg->tx.tim) //timer same
                &&  (scom->cfg->rx.channel == scom->cfg->tx.channel))//channel same
            {
                scom->irq_type = scom->tx_busy;
                return(scom);
            }
#endif     
            scom->irq_type = 0;
            return(scom);
        }
        if ((scom->cfg->tx.tim == tim) && (scom->cfg->tx.channel == channel))
        {
            scom->irq_type = 1;
            return(scom);
        }
    }

    return(RT_NULL);
}

static void ss_tim_rx_irq_deal(ss_scom_t * scom)
{
    ss_ch_cfg_t *rx_cfg = (ss_ch_cfg_t *)&(scom->cfg->rx);
    uint32_t lvl = ss_rx_pin_read(scom->cfg);
    
#ifdef SS_CHK_RX_BEGIN_BIT
    if (scom->rx_step == 0)//capture
    {
        scom->rx_step++;
        ss_tim_comp_enable(rx_cfg, ss_tim_get_cc_val(rx_cfg) + scom->bit_clk/2);
        return;
    }
    if (scom->rx_step == 1)//recv begin bit
    {
        if (lvl)//begin bit is error
        {
            ss_scom_startup_recv(scom);
        }
        else//begin bit is ok
        {
            scom->rx_step++;
            scom->rx_byte = 0;
            ss_tim_set_cc_val(rx_cfg, ss_tim_get_cc_val(rx_cfg) + scom->bit_clk);
        }
        return;
    }
#else
    if (scom->rx_step == 0)//capture
    {
        scom->rx_step += 2;
        ss_tim_comp_enable(rx_cfg, ss_tim_get_cc_val(rx_cfg) + scom->bit_clk*3/2);
        return;
    }
#endif
    if (scom->rx_step < scom->parent.config.data_bits+2)//data bits
    {
        scom->rx_step++;
        scom->rx_byte >>= 1;
        if (lvl)
        {
            scom->rx_byte |= 0x80;
        }
        if ((scom->rx_step < scom->parent.config.data_bits+2) //is not latest data bit
                || scom->parent.config.parity)//or have parity bit
        {
            ss_tim_set_cc_val(rx_cfg, ss_tim_get_cc_val(rx_cfg) + scom->bit_clk);
            return;
        }
    }
    
    ss_scom_startup_recv(scom);
    
#ifdef SS_USING_RX_PARITY
    scom->rx_parity_err = 0;
    if (scom->parent.config.parity == 1)//odd
    {
        scom->rx_parity_err = (ss_cal_parity(scom->rx_byte) == lvl) ? 0 : 1;
    }
    else if (scom->parent.config.parity == 2)//even
    {
        scom->rx_parity_err = (ss_cal_parity(scom->rx_byte) == lvl) ? 1 : 0;
    }
    if (scom->rx_parity_err)
    {
        return;
    }
#endif
    
    scom->rx_valid = 1;
    scom->rx_valid_byte = (scom->rx_byte >> (8 - scom->parent.config.data_bits));
    rt_event_send(&(scom->evt), SS_EVT_RX_DONE);
    if (scom->parent.serial_rx)
    {
        rt_hw_serial_isr((rt_serial_t *)scom, RT_SERIAL_EVENT_RX_IND);
    }
}

static void ss_tim_tx_irq_deal(ss_scom_t * scom)
{
    ss_ch_cfg_t *tx_cfg = (ss_ch_cfg_t *)&(scom->cfg->tx);
    
    if (scom->tx_step < scom->parent.config.data_bits)//send data bits
    {
        scom->tx_step++;
        ss_tx_pin_write(scom->cfg, (scom->tx_byte & 0x01));
        scom->tx_byte >>= 1;
        ss_tim_set_cc_val(tx_cfg, ss_tim_get_cc_val(tx_cfg) + scom->bit_clk);
        return;
    }
    if (scom->tx_step == scom->parent.config.data_bits)//send parity or stop
    {
        scom->tx_step++;
        if (scom->parent.config.parity == 1)//odd
        {
            ss_tx_pin_write(scom->cfg, scom->tx_parity);
            ss_tim_set_cc_val(tx_cfg, ss_tim_get_cc_val(tx_cfg) + scom->bit_clk);
            return;
        } 
        if (scom->parent.config.parity == 2)//even
        {
            ss_tx_pin_write(scom->cfg, ! scom->tx_parity);
            ss_tim_set_cc_val(tx_cfg, ss_tim_get_cc_val(tx_cfg) + scom->bit_clk);
            return;
        }
    }
    if (scom->tx_step <= scom->parent.config.data_bits + 1 + scom->parent.config.stop_bits)
    {
        scom->tx_step++;
        ss_tx_pin_write(scom->cfg, 1);//send stop bit
        ss_tim_set_cc_val(tx_cfg, ss_tim_get_cc_val(tx_cfg) + scom->bit_clk);
        return;
    }

    scom->tx_step = 0;
    scom->tx_busy = 0;
#ifdef SS_MODE_HALF
    ss_scom_startup_recv(scom);
#else
    ss_tim_cc_disable(tx_cfg);
#endif
    rt_event_send(&(scom->evt), SS_EVT_TX_DONE);
    if (scom->parent.serial_tx)
    {
        rt_hw_serial_isr((rt_serial_t *)scom, RT_SERIAL_EVENT_TX_DONE);
    }
}

static void ss_tim_irq_ch_deal(TIM_TypeDef *tim, uint32_t channel)
{
    ss_scom_t * scom = ss_tim_scom_get(tim, channel);
    
    if (scom->irq_type)//tx
    {
        ss_tim_tx_irq_deal(scom);
    }
    else//rx
    {
        ss_tim_rx_irq_deal(scom);
    }
}

static void ss_tim_irq_deal(TIM_TypeDef *tim)
{
    if (LL_TIM_IsEnabledIT_CC1(tim) && LL_TIM_IsActiveFlag_CC1(tim))
    {
        LL_TIM_ClearFlag_CC1(tim);
        ss_tim_irq_ch_deal(tim, LL_TIM_CHANNEL_CH1);
    }
    if (LL_TIM_IsEnabledIT_CC2(tim) && LL_TIM_IsActiveFlag_CC2(tim))
    {
        LL_TIM_ClearFlag_CC2(tim);
        ss_tim_irq_ch_deal(tim, LL_TIM_CHANNEL_CH2);
    }
    if (LL_TIM_IsEnabledIT_CC3(tim) && LL_TIM_IsActiveFlag_CC3(tim))
    {
        LL_TIM_ClearFlag_CC3(tim);
        ss_tim_irq_ch_deal(tim, LL_TIM_CHANNEL_CH3);
    }
    if (LL_TIM_IsEnabledIT_CC4(tim) && LL_TIM_IsActiveFlag_CC4(tim))
    {
        LL_TIM_ClearFlag_CC4(tim);
        ss_tim_irq_ch_deal(tim, LL_TIM_CHANNEL_CH4);
    }
}

#ifdef SS_USING_TIM1
void TIM1_CC_IRQHandler(void)
{
    rt_interrupt_enter();
    ss_tim_irq_deal(TIM1);
    rt_interrupt_leave();
}
#endif

#ifdef SS_USING_TIM2
void TIM2_IRQHandler(void)
{
    rt_interrupt_enter();
    ss_tim_irq_deal(TIM2);
    rt_interrupt_leave();
}
#endif

#ifdef SS_USING_TIM3
void TIM3_IRQHandler(void)
{
    rt_interrupt_enter();
    ss_tim_irq_deal(TIM3);
    rt_interrupt_leave();
}
#endif

#ifdef SS_USING_TIM4
void TIM4_IRQHandler(void)
{
    rt_interrupt_enter();
    ss_tim_irq_deal(TIM4);
    rt_interrupt_leave();
}
#endif

#ifdef SS_USING_TIM5
void TIM5_IRQHandler(void)
{
    rt_interrupt_enter();
    ss_tim_irq_deal(TIM5);
    rt_interrupt_leave();
}
#endif

#ifdef SS_USING_TIM8
void TIM8_CC_IRQHandler(void)
{
    rt_interrupt_enter();
    ss_tim_irq_deal(TIM8);
    rt_interrupt_leave();
}
#endif

static void ss_scom_init(ss_scom_t *scom)
{
    if (scom->init == 0)
    {
        scom->init = 1;
        scom->bit_clk = SS_TIM_CLK_FREQ / scom->parent.config.baud_rate;
        rt_event_init(&(scom->evt), scom->cfg->name, RT_IPC_FLAG_FIFO);
        ss_pin_init(scom->cfg);
        ss_tx_pin_write(scom->cfg, 1);
        ss_tim_init(scom->cfg->rx.tim);
        ss_tim_init(scom->cfg->tx.tim);
        ss_scom_startup_recv(scom);
    }
}

static rt_err_t ss_scom_config(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    ss_scom_t *scom = (ss_scom_t *)serial;

    scom->parent.config = *cfg;
    scom->bit_clk = SS_TIM_CLK_FREQ / scom->parent.config.baud_rate;
    ss_scom_init(scom);
    
    return RT_EOK;
}

static rt_err_t ss_scom_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    return(RT_EOK);
}

static int ss_scom_putc(struct rt_serial_device *serial, char c)
{
    RT_ASSERT(serial != RT_NULL);
    
    ss_scom_send_byte((ss_scom_t *)serial, (uint8_t)c);

    return(1);
}

static int ss_scom_getc(struct rt_serial_device *serial)
{
    RT_ASSERT(serial != RT_NULL);
    ss_scom_t *scom = (ss_scom_t *)serial;
    int ch = -1;

    if (scom->rx_valid)
    {
        scom->rx_valid = 0;
        ch = scom->rx_valid_byte;
    }

    return(ch);
}

static const struct rt_uart_ops ss_com_ops =
{
    .configure = ss_scom_config,
    .control = ss_scom_control,
    .putc = ss_scom_putc,
    .getc = ss_scom_getc,
    .dma_transmit = RT_NULL
};

int ss_scoms_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_uint32_t flag = (RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);

    for (int i = 0; i < SS_SCOM_TOTAL; i++)
    {
        ss_scom_t *scom = ss_scoms + i;
        memset((void *)scom, 0, sizeof(ss_scom_t));
        scom->cfg = ss_cfg_table + i;
        scom->parent.ops = &ss_com_ops;
        scom->parent.config = config;
        rt_hw_serial_register((rt_serial_t *)scom, scom->cfg->name, flag, RT_NULL);
    }

    return(RT_EOK);
}
INIT_BOARD_EXPORT(ss_scoms_init);

