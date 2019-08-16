/**
 ******************************************************************************
 * @file    lps22hb.h
 * @author  MEMS Software Solutions Team
 * @brief   LPS22HB header driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LPS22HB_H
#define LPS22HB_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "lps22hb_reg.h"
#include <string.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @addtogroup LPS22HB LPS22HB
 * @{
 */

/** @defgroup LPS22HB_Exported_Types LPS22HB Exported Types
 * @{
 */

typedef int32_t (*LPS22HB_Init_Func)(void);
typedef int32_t (*LPS22HB_DeInit_Func)(void);
typedef int32_t (*LPS22HB_GetTick_Func)(void);
typedef int32_t (*LPS22HB_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LPS22HB_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LPS22HB_Init_Func          Init;
  LPS22HB_DeInit_Func        DeInit;
  uint32_t                   BusType; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
  uint8_t                    Address;
  LPS22HB_WriteReg_Func      WriteReg;
  LPS22HB_ReadReg_Func       ReadReg;
  LPS22HB_GetTick_Func       GetTick;
} LPS22HB_IO_t;

typedef struct
{
  LPS22HB_IO_t        IO;
  lps22hb_ctx_t       Ctx;
  uint8_t             is_initialized;
  uint8_t             press_is_enabled;
  uint8_t             temp_is_enabled;
  lps22hb_odr_t       last_odr;
} LPS22HB_Object_t;

typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} LPS22HB_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LPS22HB_Object_t *);
  int32_t (*DeInit)(LPS22HB_Object_t *);
  int32_t (*ReadID)(LPS22HB_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LPS22HB_Object_t *, LPS22HB_Capabilities_t *);
} LPS22HB_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LPS22HB_Object_t *);
  int32_t (*Disable)(LPS22HB_Object_t *);
  int32_t (*GetOutputDataRate)(LPS22HB_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS22HB_Object_t *, float);
  int32_t (*GetTemperature)(LPS22HB_Object_t *, float *);
} LPS22HB_TEMP_Drv_t;

typedef struct
{
  int32_t (*Enable)(LPS22HB_Object_t *);
  int32_t (*Disable)(LPS22HB_Object_t *);
  int32_t (*GetOutputDataRate)(LPS22HB_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS22HB_Object_t *, float);
  int32_t (*GetPressure)(LPS22HB_Object_t *, float *);
} LPS22HB_PRESS_Drv_t;

typedef enum
{
  LPS22HB_FIFO_BYPASS_MODE                    = (uint8_t)0x00,    /*!< The FIFO is disabled and empty. The pressure is read directly*/
  LPS22HB_FIFO_FIFO_MODE                      = (uint8_t)0x20,    /*!< Stops collecting data when full */
  LPS22HB_FIFO_STREAM_MODE                    = (uint8_t)0x40,    /*!< Keep the newest measurements in the FIFO*/
  LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,    /*!< STREAM MODE until trigger deasserted, then change to FIFO MODE*/
  LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,    /*!< BYPASS MODE until trigger deasserted, then STREAM MODE*/
  LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0     /*!< BYPASS mode until trigger deasserted, then FIFO MODE*/
} LPS22HB_FifoMode;

/**
 * @}
 */

/** @defgroup LPS22HB_Exported_Constants LPS22HB Exported Constants
 * @{
 */

#define LPS22HB_OK                0
#define LPS22HB_ERROR            -1

#define LPS22HB_I2C_BUS          0U
#define LPS22HB_SPI_4WIRES_BUS   1U
#define LPS22HB_SPI_3WIRES_BUS   2U

#define LPS22HB_FIFO_FULL        (uint8_t)0x20

/**
 * @}
 */

/** @addtogroup LPS22HB_Exported_Functions LPS22HB Exported Functions
 * @{
 */

int32_t LPS22HB_RegisterBusIO(LPS22HB_Object_t *pObj, LPS22HB_IO_t *pIO);
int32_t LPS22HB_Init(LPS22HB_Object_t *pObj);
int32_t LPS22HB_DeInit(LPS22HB_Object_t *pObj);
int32_t LPS22HB_ReadID(LPS22HB_Object_t *pObj, uint8_t *Id);
int32_t LPS22HB_GetCapabilities(LPS22HB_Object_t *pObj, LPS22HB_Capabilities_t *Capabilities);
int32_t LPS22HB_Get_Init_Status(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_PRESS_Enable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_PRESS_Disable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_PRESS_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr);
int32_t LPS22HB_PRESS_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr);
int32_t LPS22HB_PRESS_GetPressure(LPS22HB_Object_t *pObj, float *Value);
int32_t LPS22HB_PRESS_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_PRESS_Get_FThStatus(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_TEMP_Enable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_TEMP_Disable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_TEMP_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr);
int32_t LPS22HB_TEMP_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr);
int32_t LPS22HB_TEMP_GetTemperature(LPS22HB_Object_t *pObj, float *Value);
int32_t LPS22HB_TEMP_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_FIFO_Get_Data(LPS22HB_Object_t *pObj, float *Press, float *Temp);
int32_t LPS22HB_FIFO_Get_FTh_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Full_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Level(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Ovr_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Reset_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt);
int32_t LPS22HB_FIFO_Set_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt);
int32_t LPS22HB_FIFO_Set_Mode(LPS22HB_Object_t *pObj, uint8_t Mode);
int32_t LPS22HB_FIFO_Set_Watermark_Level(LPS22HB_Object_t *pObj, uint8_t Watermark);
int32_t LPS22HB_FIFO_Usage(LPS22HB_Object_t *pObj, uint8_t Status);


int32_t LPS22HB_Read_Reg(LPS22HB_Object_t *pObj, uint8_t reg, uint8_t *Data);
int32_t LPS22HB_Write_Reg(LPS22HB_Object_t *pObj, uint8_t reg, uint8_t Data);

int32_t LPS22HB_Get_Press(LPS22HB_Object_t *pObj, float *Data);
int32_t LPS22HB_Get_Temp(LPS22HB_Object_t *pObj, float *Data);

/**
 * @}
 */

/** @addtogroup LPS22HB_Exported_Variables LPS22HB Exported Variables
 * @{
 */
extern LPS22HB_CommonDrv_t LPS22HB_COMMON_Driver;
extern LPS22HB_PRESS_Drv_t LPS22HB_PRESS_Driver;
extern LPS22HB_TEMP_Drv_t LPS22HB_TEMP_Driver;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
