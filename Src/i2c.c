/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"
#include "stdlib.h"
#include "string.h"
#include "gpio.h"
#include "mpu9250.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c3;

/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


extern void Delay_Us(uint32_t nus);
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);   
    
    for(int i=0;i<9;++i){
      if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)!=GPIO_PIN_SET){
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
        Delay_Us(10);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
        Delay_Us(10);
      }else{
        break;
      }
    }
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
    Delay_Us(10);

    i2cHandle->Instance->CR1=I2C_CR1_SWRST;
    i2cHandle->Instance->CR1=0;
    
  /* USER CODE END I2C3_MspInit 0 */
  
    /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Peripheral clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();

    /* I2C3 interrupt Init */
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);

  /* USER CODE BEGIN I2C3_MspInit 1 */
    
  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();
    /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    /* I2C3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

#define OUTTIME 10



uint8_t I2C_Read_Buffer(uint8_t slaveAddr,uint8_t readAddr,uint8_t *pBuffer,uint16_t len){
    uint8_t result;
	result=HAL_I2C_Master_Transmit(&hi2c3,slaveAddr,&readAddr,1,OUTTIME);
    if(result!=HAL_OK){
      return result;
    }
	return HAL_I2C_Master_Receive(&hi2c3,slaveAddr,pBuffer,len,OUTTIME);		 
}


uint8_t I2C_Write_Buffer(uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len){
	uint8_t * data;
	uint8_t result;
	data=(uint8_t *)malloc((len+1)*sizeof(uint8_t));
	memcpy(data+1,pBuffer,len);
	data[0]=writeAddr;
	result=HAL_I2C_Master_Transmit(&hi2c3,slaveAddr,data,len+1,OUTTIME);
	free(data);
	return result;
}

void I2C_Reset(){
    uint8_t temp=0;
    while(temp!=0x71){
      HAL_I2C_MspDeInit(&hi2c3);
      HAL_I2C_MspInit(&hi2c3);
      //MX_I2C3_Init();
      //MPU9250_Init(&MPU9250);
      I2C_Read_Buffer(0xD0,0x75,&temp,1);
      
    }
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
