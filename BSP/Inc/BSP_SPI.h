#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_i2s.h"
#include "FreeRTOS.h"
#include "semphr.h"

/**
 * @brief Initialize a given SPI peripheral. Struct config must be performed
 *        outside of this function.
 * 
 * @param config 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef BSP_SPI_Init(SPI_HandleTypeDef* config);
/**
 * @brief Perform a SPI write.
 * 
 * @param chip_select CS value to write to
 * @param buffer Data buffer
 * @param bytes Number of bytes being written
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef BSP_SPI_Write(uint8_t chip_select, uint8_t* buffer, uint16_t bytes);
/**
 * @brief Perform a SPI read.
 * 
 * @param buffer Return data buffer
 * @param bytes Number of bytes being read
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef BSP_SPI_Read(uint8_t* buffer, uint16_t bytes);

#endif