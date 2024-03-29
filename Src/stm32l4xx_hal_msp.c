/**
  ******************************************************************************
  * File Name          : stm32l4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
extern void _Error_Handler(char *, int);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

static uint32_t DFSDM1_Init = 0;

void HAL_DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef* hdfsdm_channel)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(DFSDM1_Init == 0)
    {
        /* USER CODE BEGIN DFSDM1_MspInit 0 */

        /* USER CODE END DFSDM1_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_DFSDM1_CLK_ENABLE();

        /**DFSDM1 GPIO Configuration
        PE7     ------> DFSDM1_DATIN2
        PE9     ------> DFSDM1_CKOUT
        */
        GPIO_InitStruct.Pin = DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* USER CODE BEGIN DFSDM1_MspInit 1 */

        /* USER CODE END DFSDM1_MspInit 1 */
    }

}

void HAL_DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef* hdfsdm_channel)
{

    DFSDM1_Init-- ;
    if(DFSDM1_Init == 0)
    {
        /* USER CODE BEGIN DFSDM1_MspDeInit 0 */

        /* USER CODE END DFSDM1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_DFSDM1_CLK_DISABLE();

        /**DFSDM1 GPIO Configuration
        PE7     ------> DFSDM1_DATIN2
        PE9     ------> DFSDM1_CKOUT
        */
        HAL_GPIO_DeInit(GPIOE, DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin);

        /* USER CODE BEGIN DFSDM1_MspDeInit 1 */

        /* USER CODE END DFSDM1_MspDeInit 1 */
    }

}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* hqspi)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(hqspi->Instance==QUADSPI)
    {
        /* USER CODE BEGIN QUADSPI_MspInit 0 */

        /* USER CODE END QUADSPI_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_QSPI_CLK_ENABLE();

        /**QUADSPI GPIO Configuration
        PE10     ------> QUADSPI_CLK
        PE11     ------> QUADSPI_NCS
        PE12     ------> QUADSPI_BK1_IO0
        PE13     ------> QUADSPI_BK1_IO1
        PE14     ------> QUADSPI_BK1_IO2
        PE15     ------> QUADSPI_BK1_IO3
        */
        GPIO_InitStruct.Pin =
            QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
            |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* USER CODE BEGIN QUADSPI_MspInit 1 */

        /* USER CODE END QUADSPI_MspInit 1 */
    }

}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* hqspi)
{

    if(hqspi->Instance==QUADSPI)
    {
        /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

        /* USER CODE END QUADSPI_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_QSPI_CLK_DISABLE();

        /**QUADSPI GPIO Configuration
        PE10     ------> QUADSPI_CLK
        PE11     ------> QUADSPI_NCS
        PE12     ------> QUADSPI_BK1_IO0
        PE13     ------> QUADSPI_BK1_IO1
        PE14     ------> QUADSPI_BK1_IO2
        PE15     ------> QUADSPI_BK1_IO3
        */
        HAL_GPIO_DeInit(GPIOE,
                        QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
                        |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin);

        /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

        /* USER CODE END QUADSPI_MspDeInit 1 */
    }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(hspi->Instance==SPI3)
    {
        /* USER CODE BEGIN SPI3_MspInit 0 */

        /* USER CODE END SPI3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_SPI3_CLK_ENABLE();

        /**SPI3 GPIO Configuration
        PC10     ------> SPI3_SCK
        PC11     ------> SPI3_MISO
        PC12     ------> SPI3_MOSI
        */
        GPIO_InitStruct.Pin =
            INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USER CODE BEGIN SPI3_MspInit 1 */

        /* USER CODE END SPI3_MspInit 1 */
    }

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

    if(hspi->Instance==SPI3)
    {
        /* USER CODE BEGIN SPI3_MspDeInit 0 */

        /* USER CODE END SPI3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI3_CLK_DISABLE();

        /**SPI3 GPIO Configuration
        PC10     ------> SPI3_SCK
        PC11     ------> SPI3_MISO
        PC12     ------> SPI3_MOSI
        */
        HAL_GPIO_DeInit(GPIOC,
                        INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin);

        /* USER CODE BEGIN SPI3_MspDeInit 1 */

        /* USER CODE END SPI3_MspDeInit 1 */
    }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

    if(htim_base->Instance==TIM7)
    {
        /* USER CODE BEGIN TIM7_MspInit 0 */

        /* USER CODE END TIM7_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM7_CLK_ENABLE();
        /* USER CODE BEGIN TIM7_MspInit 1 */

        /* USER CODE END TIM7_MspInit 1 */
    }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

    if(htim_base->Instance==TIM7)
    {
        /* USER CODE BEGIN TIM7_MspDeInit 0 */

        /* USER CODE END TIM7_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM7_CLK_DISABLE();
        /* USER CODE BEGIN TIM7_MspDeInit 1 */

        /* USER CODE END TIM7_MspDeInit 1 */
    }

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(huart->Instance==USART3)
    {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

    if(huart->Instance==USART3)
    {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOD, INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin);

        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }

}

void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(hpcd->Instance==USB_OTG_FS)
    {
        /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

        /* USER CODE END USB_OTG_FS_MspInit 0 */

        /**USB_OTG_FS GPIO Configuration
        PA9     ------> USB_OTG_FS_VBUS
        PA10     ------> USB_OTG_FS_ID
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
        */
        GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        /* Enable VDDUSB */
        if(__HAL_RCC_PWR_IS_CLK_DISABLED())
        {
            __HAL_RCC_PWR_CLK_ENABLE();
            HAL_PWREx_EnableVddUSB();
            __HAL_RCC_PWR_CLK_DISABLE();
        }
        else
        {
            HAL_PWREx_EnableVddUSB();
        }
        /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

        /* USER CODE END USB_OTG_FS_MspInit 1 */
    }

}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{

    if(hpcd->Instance==USB_OTG_FS)
    {
        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

        /* USER CODE END USB_OTG_FS_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

        /**USB_OTG_FS GPIO Configuration
        PA9     ------> USB_OTG_FS_VBUS
        PA10     ------> USB_OTG_FS_ID
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
        */
        HAL_GPIO_DeInit(GPIOA,
                        USB_OTG_FS_VBUS_Pin|USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin);

        /* Disable VDDUSB */
        if(__HAL_RCC_PWR_IS_CLK_DISABLED())
        {
            __HAL_RCC_PWR_CLK_ENABLE();
            HAL_PWREx_DisableVddUSB();
            __HAL_RCC_PWR_CLK_DISABLE();
        }
        else
        {
            HAL_PWREx_DisableVddUSB();
        }
        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

        /* USER CODE END USB_OTG_FS_MspDeInit 1 */
    }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
