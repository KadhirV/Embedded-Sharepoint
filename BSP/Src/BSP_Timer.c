/* Copyright (c) 2018-2022 UT Longhorn Racing Solar */

#include "BSP_Timer.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_rcc.h"

// define timers used here (number here only, i.e. '1' for TIM1)
#define BSP_TIMER_TICKCOUNTER        2
#define BSP_TIMER_ONESHOT            5

// some preprocessor stuff to change timers 'easily'
#define BSP_TIMER_CONCAT2_(x, y) x ## y
#define BSP_TIMER_CONCAT2(x, y) BSP_TIMER_CONCAT2_(x, y)
#define BSP_TIMER_CONCAT3_(x, y, z) x ## y ## z
#define BSP_TIMER_CONCAT3(x, y, z) BSP_TIMER_CONCAT3_(x, y, z)

#define BSP_TIMER_INST(timer_num) BSP_TIMER_CONCAT2(TIM, timer_num)
#define BSP_TIMER_RCC(timer_num) BSP_TIMER_CONCAT2(RCC_APB1Periph_TIM, timer_num)
#define BSP_TIMER_IRQ(timer_num) BSP_TIMER_CONCAT3(TIM, timer_num, _IRQHandler)
#define BSP_TIMER_IRQn(timer_num) BSP_TIMER_CONCAT3(TIM, timer_num, _IRQn)

// global constants
static const uint32_t MICROSECONDS_PER_SECOND = (int)1e6;
static uint32_t TimerFrequency = 0;

// globals
static callback_t TimerOneShotCallback;

/**
 * @brief little helper function to find valid timer period and prescaler values 
 *        for a given timer period. maximizes the period and minimizes the prescaler.
 * @param delay_us timer period in microseconds
 * @param period period will be placed here
 * @param prescaler prescaler will be placed here
 * @param timer_32bit true if 32 bit timer (TIM2/TIM5), false if 16 bit timer (other timers)
 * @return error (in clock cycles) betweeen requested period and actual period,
 *         -1 if requested period is too large
 */
static uint32_t Timer_Micros_To_PeriodPrescaler(uint32_t delay_us, 
                                                uint32_t *period, 
                                                uint16_t *prescaler,
                                                bool timer_32bit) {
    uint32_t delay_clock_cycles = (TimerFrequency / MICROSECONDS_PER_SECOND) * delay_us;
    uint32_t prescale_value = 1, period_value = delay_clock_cycles;

    while (period_value > (timer_32bit ? UINT32_MAX : UINT16_MAX)) {
        prescale_value <<= 1;
        period_value >>= 1;
    }
    *period = period_value;
    *prescaler = (uint16_t)(prescale_value - 1);
    return (prescale_value > UINT16_MAX) ? -1 : (delay_clock_cycles - (period_value * prescale_value));
}

/**
 * @brief   Initialize timers
 * @param   None
 * @return  None
 */
void BSP_Timer_Init(void) {
    
    
    // enable clock(s)
    if (__HAL_RCC_TIM2_IS_CLK_DISABLED()){
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
    if (__HAL_RCC_TIM5_IS_CLK_DISABLED()){
        __HAL_RCC_TIM5_CLK_ENABLE();
    }

    RCC_ClkInitTypeDef RCC_Clocks;
    HAL_RCC_ClockConfig(&RCC_Clocks, FLASH_ACR_LATENCY);
    TimerFrequency = RCC_Clocks.APB2CLKDivider;

    TIM_Base_InitTypeDef timer_tickcounter;
    TIM_Base_SetConfig(, &timer_tickcounter); 

    TIM_Base_InitTypeDef timer_oneshot;
    TIM_Base_SetConfig(, &timer_oneshot);

    TIM_TimeBaseInit(BSP_TIMER_INST(BSP_TIMER_TICKCOUNTER), &timer_tickcounter);
    TIM_TimeBaseInit(BSP_TIMER_INST(BSP_TIMER_ONESHOT), &timer_oneshot);

    // one shot nvic initialization
    NVIC_InitTypeDef nvic_timer_oneshot = {
        .NVIC_IRQChannel = BSP_TIMER_IRQn(BSP_TIMER_ONESHOT),
        .NVIC_IRQChannelPreemptionPriority = 2,
        .NVIC_IRQChannelSubPriority = 1,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&nvic_timer_oneshot);

}

/**
 * @brief   Starts a one shot timer to execute a callback after a certain time
 * 
 * @param delay_us one shot time in microseconds
 * @param callback callback to execute after `delay_us` time
 * 
 * @note Calling this multiple times concurrently results in undefined behavior.
 *       Define more one shot timers if >1 are needed concurrently. 
 *       Look at RTOS_BPS_DelayUs() for an example use case
 */
void BSP_Timer_Start_OneShot(uint32_t delay_us, callback_t callback) {
    TIM_TypeDef *tim_inst = BSP_TIMER_INST(BSP_TIMER_ONESHOT);

    TimerOneShotCallback = callback;
    uint32_t period;
    uint16_t prescaler;
    Timer_Micros_To_PeriodPrescaler(delay_us, &period, &prescaler, true);
    
    TIM_SetAutoreload(tim_inst, period);
    TIM_PrescalerConfig(tim_inst, prescaler, TIM_PSCReloadMode_Immediate);

    TIM_Cmd(tim_inst, ENABLE);

    TIM_SetCounter (tim_inst, 0);
    TIM_ClearITPendingBit (tim_inst, TIM_IT_Update);
    TIM_ITConfig(tim_inst, TIM_IT_Update, ENABLE);
    
}

/**
 * @brief   Starts the tick counter timer
 * @param   None
 * @return  None
 */
void BSP_Timer_Start_TickCounter(void) {
    TIM_TypeDef *tim_inst = BSP_TIMER_INST(BSP_TIMER_TICKCOUNTER);
    TIM_Cmd(tim_inst, ENABLE);
}

/**
 * @brief   Gets the number of ticks that has elapsed since the last time this function was called.
 * @param   None
 * @return  Number of ticks
 */
uint32_t BSP_Timer_GetTicksElapsed(void) {
    /* Get system clocks */
    uint32_t counter = TIM2->CNT;       // find current value of up counter
    TIM2->CNT = 0;                      // reset up counter

    return counter;
}

/**
 * @brief   Gets the running frequency of the timer (time per tick)
 * @param   None
 * @return  frequency in Hz
 */
uint32_t BSP_Timer_GetRunFreq(void) {
    return TimerFrequency;
}

/**
 * @brief   Gives a standard unit for time elapsed in microseconds since calling BSP_Timer_GetTicksElapsed()
 * @param   None
 * @return  Microseconds 
 */
uint32_t BSP_Timer_GetMicrosElapsed(void) {
    uint32_t ticks = BSP_Timer_GetTicksElapsed();
    uint32_t freq = BSP_Timer_GetRunFreq();
    uint32_t micros_elap = ticks / (freq / MICROSECONDS_PER_SECOND); // Math to ensure that we do not overflow (16Mhz or 80Mhz)
    return micros_elap;
} 

extern void BSP_TIMER_IRQ(BSP_TIMER_ONESHOT)() {
    TIM_TypeDef *tim_inst = BSP_TIMER_INST(BSP_TIMER_ONESHOT);
    if (TIM_GetITStatus(tim_inst, TIM_IT_Update) != RESET) {
        // disable timer
        TIM_ITConfig(BSP_TIMER_INST(BSP_TIMER_ONESHOT), TIM_IT_Update, DISABLE);
        TIM_Cmd(tim_inst, DISABLE);

        TIM_ClearITPendingBit(tim_inst, TIM_IT_Update);

        TimerOneShotCallback();
    }
}