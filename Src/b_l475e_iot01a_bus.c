/**
  ******************************************************************************
  * @file           : b_l475e_iot01a_bus.h
  * @brief          : source file for the BSP BUS IO driver
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
#include "b_l475e_iot01a_bus.h"
#include "b_l475e_iot01a_errno.h"
#include "stm32l4xx_hal.h"

#define TIMEOUT_DURATION 1000
/** @addtogroup BSP
  * @{
  */
__weak HAL_StatusTypeDef MX_I2C2_Init(I2C_HandleTypeDef* hi2c);

/** @addtogroup B_L475E_IOT01A
  * @{
  */

/** @defgroup B_L475E_IOT01A_BUS B_L475E_IOT01A BUS
  * @{
  */

/** @defgroup B_L475E_IOT01A_Private_Variables BUS Private Variables
  * @{
  */
I2C_HandleTypeDef hbusi2c2;
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
static uint32_t IsI2C2MspCbValid = 0;
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup B_L475E_IOT01A_Private_FunctionPrototypes  Private Function Prototypes
  * @{
  */

static void I2C2_MspInit(I2C_HandleTypeDef* i2cHandle);
static void I2C2_MspDeInit(I2C_HandleTypeDef* i2cHandle);

/**
  * @}
  */

/** @defgroup B_L475E_IOT01A_LOW_LEVEL_Private_Functions B_L475E_IOT01A LOW LEVEL Private Functions
  * @{
  */

/** @defgroup B_L475E_IOT01A_BUS_Exported_Functions B_L475E_IOT01A_BUS Exported Functions
  * @{
  */
/* BUS IO driver over I2C Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER I2C
*******************************************************************************/
/**
  * @brief  Initialize a bus
  * @param None
  * @retval BSP status
  */
int32_t BSP_I2C2_Init(void)
{

    int32_t ret = BSP_ERROR_NONE;

    hbusi2c2.Instance  = I2C2;

    if(HAL_I2C_GetState(&hbusi2c2) == HAL_I2C_STATE_RESET)
    {
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
        /* Init the I2C Msp */
        I2C2_MspInit(&hbusi2c2);
#else
        if(IsI2C2MspCbValid == 0U)
        {
            if(BSP_I2C2_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
            {
                return BSP_ERROR_MSP_FAILURE;
            }
        }
#endif

        /* Init the I2C */
        if(MX_I2C2_Init(&hbusi2c2) != HAL_OK)
        {
            ret = BSP_ERROR_BUS_FAILURE;
        }
        else if(HAL_I2CEx_ConfigAnalogFilter(&hbusi2c2,
                                             I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        {
            ret = BSP_ERROR_BUS_FAILURE;
        }
        else
        {
            ret = BSP_ERROR_NONE;
        }
    }

    return ret;
}

/**
  * @brief  DeInitialize a bus
  * @param None
  * @retval BSP status
  */
int32_t BSP_I2C2_DeInit(void)
{
    int32_t ret = BSP_ERROR_BUS_FAILURE;

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
    /* DeInit the I2C */
    I2C2_MspDeInit(&hbusi2c2);
#endif

    if(HAL_I2C_DeInit(&hbusi2c2) == HAL_OK)
    {
        ret = BSP_ERROR_NONE;
    }

    return ret;
}

/**
  * @brief Return the status of the Bus
  *	@retval bool
  */
int32_t BSP_I2C2_IsReady(void)
{
    return (HAL_I2C_GetState(&hbusi2c2) == HAL_I2C_STATE_READY);
}

/**
  * @brief  Write registers through bus (8 bits)
  * @param  Addr: Device address on Bus.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  * @retval BSP status
  */
int32_t BSP_I2C2_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData,
                          uint16_t len)
{
    int32_t ret = BSP_ERROR_BUS_FAILURE;

    if(HAL_I2C_Mem_Write(&hbusi2c2, (uint8_t)DevAddr,
                         (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,
                         (uint8_t *)pData, len, TIMEOUT_DURATION) == HAL_OK)
    {
        ret = BSP_ERROR_NONE;
    }

    return ret;
}

/**
  * @brief  Read registers through a bus (8 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @retval BSP status
  */
int32_t  BSP_I2C2_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData,
                          uint16_t len)
{
    int32_t ret = BSP_ERROR_BUS_FAILURE;

    if(HAL_I2C_Mem_Read(&hbusi2c2, DevAddr, (uint16_t)Reg,
                        I2C_MEMADD_SIZE_8BIT, pData,
                        len, TIMEOUT_DURATION) == HAL_OK)
    {
        ret = HAL_OK;
    }

    return ret;
}

/**
  * @brief  Write registers through bus (16 bits)
  * @param  Addr: Device address on Bus.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  * @retval BSP status
  */
int32_t BSP_I2C2_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData,
                            uint16_t len)
{
    int32_t ret = BSP_ERROR_BUS_FAILURE;

    if(HAL_I2C_Mem_Write(&hbusi2c2, (uint8_t)DevAddr,
                         (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT,
                         (uint8_t *)pData, len, TIMEOUT_DURATION) == HAL_OK)
    {
        ret = BSP_ERROR_NONE;
    }

    return ret;
}

/**
  * @brief  Read registers through a bus (16 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @retval BSP status
  */
int32_t  BSP_I2C2_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData,
                            uint16_t len)
{
    int32_t ret = BSP_ERROR_BUS_FAILURE;

    if(HAL_I2C_Mem_Read(&hbusi2c2, DevAddr, (uint16_t)Reg,
                        I2C_MEMADD_SIZE_16BIT, pData,
                        len, TIMEOUT_DURATION) == HAL_OK)
    {
        ret = BSP_ERROR_NONE;
    }

    return ret;
}

/**
  * @brief  Send an amount width data through bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C2_Send(uint16_t DevAddr, uint8_t *pData, uint16_t len)
{
    int32_t ret = BSP_ERROR_BUS_FAILURE;

    if(HAL_I2C_Master_Transmit(&hbusi2c2, DevAddr, pData, len,
                               TIMEOUT_DURATION) == HAL_OK)
    {
        ret = len;
    }

    return ret;
}

/**
  * @brief  Receive an amount of data through a bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C2_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t len)
{
    int32_t ret = BSP_ERROR_BUS_FAILURE;

    if(HAL_I2C_Master_Receive(&hbusi2c2, DevAddr, pData, len,
                              TIMEOUT_DURATION) == HAL_OK)
    {
        ret = len;
    }

    return ret;
}

/**
  * @brief  Send and receive an amount of data through bus (Full duplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pTxdata: Transmit data pointer
  * @param 	pRxdata: Receive data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C2_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata,
                          uint16_t len)
{
    int32_t ret = BSP_ERROR_BUS_FAILURE;

    /*
     * Send and receive an amount of data through bus (Full duplex)
     * I2C is Half-Duplex protocol
     */
    if((BSP_I2C2_Send(DevAddr, pTxdata, len) == len) && \
            (BSP_I2C2_Recv(DevAddr, pRxdata, len) == len))
    {
        ret = len;
    }

    return ret;
}

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
/**
  * @brief Register Default BSP I2C2 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_I2C2_RegisterDefaultMspCallbacks(void)
{

    __HAL_I2C_RESET_HANDLE_STATE(&hbusi2c2);

    /* Register MspInit Callback */
    if(HAL_I2C_RegisterCallback(&hbusi2c2, HAL_I2C_MSPINIT_CB_ID,
                                I2C2_MspInit)  != HAL_OK)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    /* Register MspDeInit Callback */
    if(HAL_I2C_RegisterCallback(&hbusi2c2, HAL_I2C_MSPDEINIT_CB_ID,
                                I2C2_MspDeInit) != HAL_OK)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }
    IsI2C2MspCbValid = 1;

    return BSP_ERROR_NONE;
}

/**
  * @brief BSP I2C2 Bus Msp Callback registering
  * @param Callbacks     pointer to I2C2 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_I2C2_RegisterMspCallbacks(BSP_I2C_Cb_t *Callbacks)
{
    /* Prevent unused argument(s) compilation warning */
    __HAL_I2C_RESET_HANDLE_STATE(&hbusi2c2);

    /* Register MspInit Callback */
    if(HAL_I2C_RegisterCallback(&hbusi2c2, HAL_I2C_MSPINIT_CB_ID,
                                Callbacks->pMspI2cInitCb)  != HAL_OK)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    /* Register MspDeInit Callback */
    if(HAL_I2C_RegisterCallback(&hbusi2c2, HAL_I2C_MSPDEINIT_CB_ID,
                                Callbacks->pMspI2cDeInitCb) != HAL_OK)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    IsI2C2MspCbValid = 1;

    return BSP_ERROR_NONE;
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @brief  Return system tick in ms
  * @retval Current HAL time base time stamp
  */
int32_t BSP_GetTick(void)
{
    return HAL_GetTick();
}

/* I2C2 init function */

__weak HAL_StatusTypeDef MX_I2C2_Init(I2C_HandleTypeDef* hi2c)
{
    HAL_StatusTypeDef ret = HAL_OK;
    hi2c->Instance = I2C2;
    hi2c->Init.Timing = 0x10909CEC;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(hi2c) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    if(HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    if(HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    return ret;
}

static void I2C2_MspInit(I2C_HandleTypeDef* i2cHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* USER CODE BEGIN I2C2_MspInit 0 */

    /* USER CODE END I2C2_MspInit 0 */

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = INTERNAL_I2C2_SCL_Pin|INTERNAL_I2C2_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
    /* USER CODE BEGIN I2C2_MspInit 1 */

    /* USER CODE END I2C2_MspInit 1 */
}

static void I2C2_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
    /* USER CODE BEGIN I2C2_MspDeInit 0 */

    /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, INTERNAL_I2C2_SCL_Pin|INTERNAL_I2C2_SDA_Pin);

    /* USER CODE BEGIN I2C2_MspDeInit 1 */

    /* USER CODE END I2C2_MspDeInit 1 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
