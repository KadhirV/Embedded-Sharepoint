#include "BSP_PWM.h"

// init structs
static GPIO_InitTypeDef gpio_struct;

static TIM_OC_InitTypeDef timer_oc_struct;
static TIM_HandleTypeDef timer3_handle;
static TIM_HandleTypeDef timer8_handle;
static TIM_HandleTypeDef timer12_handle;

// https://visualgdb.com/tutorials/arm/stm32/timers/hal/

/**
 * @brief   Initialize all the GPIO pins meant for pwm
 * @param   None
 * @return  None
 */
BSP_Error BSP_PWM_Init(void) {

    BSP_Error pwm_init_status;

    // enable timer 3, 8, 12 if they are disabled
    if (__HAL_RCC_TIM3_IS_CLK_DISABLED()) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
    if (__HAL_RCC_TIM8_IS_CLK_DISABLED()) {
        __HAL_RCC_TIM8_CLK_ENABLE();
    }
    if (__HAL_RCC_TIM12_IS_CLK_DISABLED()) {
        __HAL_RCC_TIM12_CLK_ENABLE();
    }
    // enable port A/B/C clocks, if disabled
    if (__HAL_RCC_GPIOA_IS_CLK_DISABLED()) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    if (__HAL_RCC_GPIOB_IS_CLK_DISABLED()) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    if (__HAL_RCC_GPIOC_IS_CLK_DISABLED()) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }

    // Configure timer pins by configuring the corresponding GPIO pins
    // Unfortunately HAL doesn't have a quick function to do GPIO_PinAFConfig
    gpio_struct.Pin = GPIO_PIN_0;
    gpio_struct.Alternate = GPIO_AF2_TIM3;
    gpio_struct.Mode = GPIO_MODE_AF_PP;
    gpio_struct.Speed = GPIO_SPEED_FAST;
    gpio_struct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &gpio_struct); // B0 = timer 3
    // C6 and C7 are timer 8
    gpio_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio_struct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio_struct);
    // B14 and B15 are timer 12
    gpio_struct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    gpio_struct.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOB, &gpio_struct);

    // gpio stuff done, configure timer blocks themselves now
    timer3_handle.Init.Period = PWM_PERIOD;
    timer3_handle.Init.Prescaler = 0;
    timer3_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer3_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer3_handle.Init.RepetitionCounter = 0;

    timer8_handle = timer12_handle = timer3_handle; // Same config EXCEPT for instances
    timer3_handle.Instance = TIM3;
    timer8_handle.Instance = TIM8;
    timer12_handle.Instance = TIM12;

    // timer 3
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_Base_Init(&timer3_handle));
    // timer 8
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_Base_init(&timer8_handle));
    // timer 12
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_Base_init(&timer12_handle));

    // oc struct init
    
    // enable the timer blocks
    if (__HAL_RCC_TIM3_IS_CLK_DISABLED()) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
    if (__HAL_RCC_TIM8_IS_CLK_DISABLED()) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
    if (__HAL_RCC_TIM12_IS_CLK_DISABLED()) {
        __HAL_RCC_TIM12_CLK_ENABLE();
    }
    // set the config for the timers
    timer_oc_struct.OCMode = TIM_OCMODE_PWM1;
    timer_oc_struct.OCIdleState = TIM_OCIDLESTATE_RESET;
    timer_oc_struct.Pulse = 0;
    timer_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    timer_oc_struct.OCPolarity = TIM_OCPOLARITY_HIGH;
    timer_oc_struct.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    // BPS timer config
    // TODO: can this be cleaned up? Unsure which HAL_TIM_XXX functions we need to use...
    // Look into ConfigChannel vs just Init() ****
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_ConfigChannel(&timer3_handle, &timer_oc_struct, TIM_CHANNEL_3));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_ConfigChannel(&timer8_handle, &timer_oc_struct, TIM_CHANNEL_1));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_ConfigChannel(&timer8_handle, &timer_oc_struct, TIM_CHANNEL_2));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_ConfigChannel(&timer12_handle, &timer_oc_struct, TIM_CHANNEL_1));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_ConfigChannel(&timer12_handle, &timer_oc_struct, TIM_CHANNEL_2));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_Start(&timer3_handle, TIM_CHANNEL_3));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_Start(&timer8_handle, TIM_CHANNEL_1));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_Start(&timer8_handle, TIM_CHANNEL_2));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_Start(&timer12_handle, TIM_CHANNEL_1));
    pwm_init_status |= CONVERT_RETURN(HAL_TIM_PWM_Start(&timer12_handle, TIM_CHANNEL_2));
    // PWM init is done

    // For contactor
    GPIO_InitTypeDef GPIO_B1Init;
    GPIO_B1Init.Pin = GPIO_PIN_1; //input pin is gpio B1
    GPIO_B1Init.Mode = MODE_INPUT;
    GPIO_B1Init.Speed = GPIO_SPEED_FAST;
    GPIO_B1Init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_B1Init);


    return pwm_init_status;
}

/**
 * @brief   Sets a pin's duty cycle
 * @param   dutyCycle: duty cycle amount between 0 and the period
 * @param   pin: pin number whose speed should be changed
 * @return  ErrorStatus
 */
BSP_Error BSP_PWM_Set(uint8_t pin, uint32_t speed) {
    // new PWM must be within range
    speed = (speed > PWM_PERIOD) ? PWM_PERIOD : speed;
    switch (pin) {
        case 0:
            __HAL_TIM_SET_COMPARE(&timer8_handle, TIM_CHANNEL_1, speed);
            break;
        case 1:
            __HAL_TIM_SET_COMPARE(&timer8_handle, TIM_CHANNEL_2, speed);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&timer12_handle, TIM_CHANNEL_1, speed);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&timer12_handle, TIM_CHANNEL_1, speed);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&timer3_handle, TIM_CHANNEL_3, speed);
            break;
        default:
            return BSP_ERROR;
            break;
    }
    return BSP_ERROR;
}

/**
 * @brief   Get current duty cycle of a single pin, return -1 if input is invalid
 * @param   pin Number
 * @return  Current PWM duty cycle of pin
 */
int BSP_PWM_Get(uint8_t pin) {
    int dutyCycle;
    switch (pin) {
        case 0:
            dutyCycle = __HAL_TIM_GET_COMPARE(&timer8_handle, TIM_CHANNEL_1);
            break;
        case 1:
            dutyCycle = __HAL_TIM_GET_COMPARE(&timer8_handle, TIM_CHANNEL_2);
            break;
        case 2:
            dutyCycle = __HAL_TIM_GET_COMPARE(&timer12_handle, TIM_CHANNEL_1);
            break;
        case 3:
            dutyCycle = __HAL_TIM_GET_COMPARE(&timer12_handle, TIM_CHANNEL_1);
            break;
        case 4:
            dutyCycle = __HAL_TIM_GET_COMPARE(&timer3_handle, TIM_CHANNEL_3);
            break;
        default:
            dutyCycle = -1;
            break;
    }
    return dutyCycle;
}

/**
 * @brief   Gets the state of the Contactor switch from one of its AUX pins.
 * @note	THIS IS ALSO CODE THAT HAS NO HOME. You cannot get the state of ALL_CONTACTORS. As such, if that param is passed, it will return the state of the array contactor.
 * @param   Contactor to get state of
 * @return  0 if contactor is off/open, 1 if on/closed
 */
bool BSP_Contactor_Get(uint8_t contactorChoice) {
    bool contactorReturnValue = ((GPIOB->IDR & GPIO_PIN_1) >> 1) ? 0 : 1; //read the one and only input pin
    return contactorReturnValue;
}