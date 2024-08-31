#include "BSP_SPI.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"

static SPI_HandleTypeDef spi_context;

/**
 * @brief Initialize a given SPI peripheral. Struct config must be performed
 *        outside of this function.
 * 
 * @param config 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef BSP_SPI_Init(SPI_HandleTypeDef* config) {
    HAL_StatusTypeDef stat;

    spi_context = *config; // going to keep track of initialized SPI locally

    // Check required clocks
    // We are only allowing SPI2/3
    if (config->Instance == SPI2) {
        if (__HAL_RCC_SPI2_IS_CLK_DISABLED()) {
            __HAL_RCC_SPI2_CLK_ENABLE();
        }
    }
    else if (config->Instance == SPI3) {
        if (__HAL_RCC_SPI3_IS_CLK_DISABLED()) {
            __HAL_RCC_SPI3_CLK_ENABLE();
        }
    } else {
        return HAL_ERROR;
    }

    stat = HAL_SPI_Init(config);

    if (stat == HAL_OK) {
        // Enable the interrupts and priorities
        if (config->Instance == SPI2) {
            HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SPI2_IRQn);
        } else if (config->Instance == SPI3) {
            HAL_NVIC_SetPriority(SPI3_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SPI3_IRQn);
        }
        __HAL_SPI_ENABLE(config); // manually enable the peripheral
    }

    return stat;
}
/**
 * @brief Perform a SPI write.
 * 
 * @param chip_select CS value to write to
 * @param buffer Data buffer
 * @param bytes Number of bytes being written
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef BSP_SPI_Write(uint8_t chip_select, uint8_t* buffer, uint16_t bytes) {
    HAL_StatusTypeDef stat;

    return stat;
}
/**
 * @brief Perform a SPI read.
 * 
 * @param buffer Return data buffer
 * @param bytes Number of bytes being read
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef BSP_SPI_Read(uint8_t* buffer, uint16_t bytes) {
    HAL_StatusTypeDef stat;

    return stat;
}