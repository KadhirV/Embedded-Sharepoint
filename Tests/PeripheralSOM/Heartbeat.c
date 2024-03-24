#include <stdio.h>
#include "stm32f4xx_hal.h" // to do: change to L4

#define TEXT_BUFFER_SIZE 20

GPIO_InitTypeDef gpio;
UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    Error_Handler();
  }
}

volatile uint8_t text_buffer[TEXT_BUFFER_SIZE] = {0}

int main(void){
    HAL_Init();

    MX_USART1_UART_Init();

    // Defines Pin B5 on the PeripheralSOM as an output
    GPIO_InitTypeDef gpio;
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP; 
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &gpio);

    sprintf(text_buffer , "Hello world\n\r");
    while (1) {

        HAL_UART_Transmit(&huart1, txt_buf, sizeof(txt_buf), 100); // print to serial terminal
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);  // Toggle the LED
        HAL_Delay(500);  // Delay for 500 milliseconds
  }

}
