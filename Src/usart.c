/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "base.h"
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include <stdarg.h>
#include "command.h"
char buffer_rx_temp;

char buffer_rx[30];
char GPS_Rx[100];

int buffer_rx_count=0;

char buffer_rx_OK;

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

DMA_HandleTypeDef hdma_usart2_rx;
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

   HAL_NVIC_SetPriority(USART2_IRQn, 4, 0);
   HAL_NVIC_EnableIRQ(USART2_IRQn);
    //开启空闲中断
   __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
   HAL_UART_Receive_DMA(&huart2, (uint8_t *)GPS_Rx, 99);
  
}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_NVIC_SetPriority(USART6_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    HAL_UART_Receive_IT(&huart6,(uint8_t *)&buffer_rx_temp,1);

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);
    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);;
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();
  
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    HAL_UART_Receive_IT(&huart6,(uint8_t *)&buffer_rx_temp,1);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();
  
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
char uart_buffer[100 + 1];
void uprintf(char *fmt, ...)
{
	int size;
	
	va_list arg_ptr;
	
	va_start(arg_ptr, fmt);  
	
	size=vsnprintf(uart_buffer, 100 + 1, fmt, arg_ptr);
	va_end(arg_ptr);
    HAL_UART_Transmit(&huart6,(uint8_t *)uart_buffer, size,1000);
	
}

extern void end(int arg_num,char **s,float *arg);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART6){
      /*
        if(buffer_rx_temp=='z'){
           end(0,0,0);
           HAL_UART_Receive_IT(&huart6,&buffer_rx_temp,1);
           return ;
        }
      */
		buffer_rx[buffer_rx_count]=buffer_rx_temp;

		if(buffer_rx[buffer_rx_count-1]=='\r'&&buffer_rx[buffer_rx_count]=='\n'){
			buffer_rx_OK=1;
			buffer_rx[buffer_rx_count-1]='\0';
			buffer_rx_count=0;
		}else{
			buffer_rx_count++;
            HAL_UART_Receive_IT(&huart6,(uint8_t *)&buffer_rx_temp,1);
		}
	}
}

/* USER CODE END 1 */

void HAL_UART_IDLECallback(UART_HandleTypeDef *huart){
  uint8_t temp;
  GPRMC GPS;
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  /*
  if(huart->Instance==USART6){
    HAL_UART_DMAStop(&huart6);
    analize(buffer_rx);
    
    temp= huart->Instance->SR;
    temp= huart->Instance->DR;//读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
    huart->hdmarx->XferCpltCallback(huart->hdmarx);
    HAL_UART_Receive_DMA(&huart6,(uint8_t *) buffer_rx, 30);
  }else 
   */ 
  if(huart->Instance==USART2){
    HAL_UART_DMAStop(&huart2);
    
    //处理GPS数据
    if(Analize_GPS(GPS_Rx,&GPS)){
      uprintf("%f \r\n%f \r\n",GPS.Lat,GPS.Long);
    }
    temp= huart->Instance->SR;
    temp= huart->Instance->DR;//读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
    
    huart->hdmarx->XferCpltCallback(huart->hdmarx);
    
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)GPS_Rx, 99);
  }
  
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
