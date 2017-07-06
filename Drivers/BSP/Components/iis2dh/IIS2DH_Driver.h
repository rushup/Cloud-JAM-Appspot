/**
 ******************************************************************************
 * @file    IIS2DH_driver.h
 * @author  MP
 * @version V1.0.0
 * @date    30-June-2016
 * @brief   This file contains definitions for the IIS2DH_Driver.c firmware 
 *          driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#ifndef __IIS2DH_DRIVER__H
#define __IIS2DH_DRIVER__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported common structure --------------------------------------------------------*/

#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef union
{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} Type3Axis16bit_U;

typedef union
{
  int16_t i16bit;
  uint8_t u8bit[2];
} Type1Axis16bit_U;

typedef union
{
  int32_t i32bit;
  uint8_t u8bit[4];
} Type1Axis32bit_U;

#endif /*__SHARED__TYPES*/


/**
* @brief  Error type.
*/
typedef enum 
{
  IIS2DH_OK = (uint8_t)0, 
  IIS2DH_ERROR = !IIS2DH_OK
} IIS2DH_Status_t;


/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/************** I2C Address *****************/

#define IIS2DH_I2C_ADDRESS_LOW   0x30  // SAD[0] = 0
#define IIS2DH_I2C_ADDRESS_HIGH  0x32  // SAD[0] = 1

/************** Who am I  *******************/

#define IIS2DH_WHO_AM_I         0x33
IIS2DH_Status_t IIS2DH_R_WHO_AM_I(void *handle, uint8_t *value);

/* Private Function Prototype -------------------------------------------------------*/

void IIS2DH_SwapHighLowByte(uint8_t *bufferToSwap, uint8_t numberOfByte, uint8_t dimension);
IIS2DH_Status_t IIS2DH_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
IIS2DH_Status_t IIS2DH_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );


/************** Device Register  *******************/

#define IIS2DH_STATUS_REG_AUX                           0x07
#define IIS2DH_OUT_TEMP_L                               0x0C
#define IIS2DH_OUT_TEMP_H                               0x0D
#define IIS2DH_INT_COUNTER_REG                          0x0E
#define IIS2DH_WHO_AM_I_REG                             0x0F
#define IIS2DH_TEMP_CFG_REG                             0x1F
#define IIS2DH_CTRL_REG1                                0x20
#define IIS2DH_CTRL_REG2                                0x21
#define IIS2DH_CTRL_REG3                                0x22
#define IIS2DH_CTRL_REG4                                0x23
#define IIS2DH_CTRL_REG5                                0x24
#define IIS2DH_CTRL_REG6                                0x25
#define IIS2DH_REFERENCE_DATACAPTURE                    0x26
#define IIS2DH_STATUS_REG                               0x27
#define IIS2DH_OUT_X_L                                  0x28
#define IIS2DH_OUT_X_H                                  0x29
#define IIS2DH_OUT_Y_L                                  0x2A
#define IIS2DH_OUT_Y_H                                  0x2B
#define IIS2DH_OUT_Z_L                                  0x2C
#define IIS2DH_OUT_Z_H                                  0x2D
#define IIS2DH_FIFO_CTRL_REG                            0x2E
#define IIS2DH_FIFO_SRC_REG                             0x2F
#define IIS2DH_INT1_CFG                                 0x30
#define IIS2DH_INT1_SRC                                 0x31
#define IIS2DH_INT1_THS                                 0x32
#define IIS2DH_INT1_DURATION                            0x33
#define IIS2DH_INT2_CFG                                 0x34
#define IIS2DH_INT2_SRC                                 0x35
#define IIS2DH_INT2_THS                                 0x36
#define IIS2DH_INT2_DURATION                            0x37
#define IIS2DH_CLICK_CFG                                0x38
#define IIS2DH_CLICK_SRC                                0x39
#define IIS2DH_CLICK_THS                                0x3A
#define IIS2DH_TIME_LIMIT                               0x3B
#define IIS2DH_TIME_LATENCY                             0x3C
#define IIS2DH_TIME_WINDOW                              0x3D
#define IIS2DH_ACT_THS                                  0x3E
#define IIS2DH_ACT_DUR                                  0x3F



typedef enum
{
  IIS2DH_FIFO_MODE_BYPASS               = (uint8_t)0x00,
  IIS2DH_FIFO_MODE_FIFO                 = (uint8_t)0x40,
  IIS2DH_FIFO_MODE_STREAM               = (uint8_t)0x80,
  IIS2DH_FIFO_MODE_STREAM_TO_FIFO       = (uint8_t)0xC0
} IIS2DH_FIFO_MODE_t;

#define IIS2DH_FIFO_MODE_MASK           (uint8_t)0xC0
IIS2DH_Status_t IIS2DH_W_FIFO_MODE(void *handle, IIS2DH_FIFO_MODE_t newValue);
IIS2DH_Status_t IIS2DH_R_FIFO_MODE(void *handle, IIS2DH_FIFO_MODE_t *value);

typedef enum
{
  IIS2DH_NORMAL_MODE                    = (uint8_t)0x00,
  IIS2DH_HIGH_RES_MODE                  = (uint8_t)0x01,
  IIS2DH_LOW_PWR_MODE                   = (uint8_t)0x02
} IIS2DH_OPER_MODE_t;

#define IIS2DH_OPER_MODE_MASK           (uint8_t)0x08
IIS2DH_Status_t IIS2DH_W_OpMode(void *handle, IIS2DH_OPER_MODE_t newValue);
IIS2DH_Status_t IIS2DH_R_OpMode(void *handle, IIS2DH_OPER_MODE_t *value);

typedef enum
{
  IIS2DH_XDA_NOT_AVAILABLE              = 0x00,
  IIS2DH_XDA_AVAILABLE                  = 0x01,
} IIS2DH_XDA_t;

#define IIS2DH_XDA_MASK                 (uint8_t)0x01
IIS2DH_Status_t IIS2DH_R_XDataAvail(void *handle, IIS2DH_XDA_t *value);

typedef enum
{
  IIS2DH_ODR_PD                         = (uint8_t)0x00,
  IIS2DH_ODR_1HZ                        = (uint8_t)0x10,
  IIS2DH_ODR_10HZ                       = (uint8_t)0x20,
  IIS2DH_ODR_25HZ                       = (uint8_t)0x30,
  IIS2DH_ODR_50HZ                       = (uint8_t)0x40,
  IIS2DH_ODR_100HZ                      = (uint8_t)0x50,
  IIS2DH_ODR_200HZ                      = (uint8_t)0x60,
  IIS2DH_ODR_400HZ                      = (uint8_t)0x70,
  IIS2DH_ODR_1620HZ_LP                  = (uint8_t)0x80,
  IIS2DH_ODR_5376HZ_LP_or_1344HZ_NM_HR  = (uint8_t)0x90
} IIS2DH_ODR_t;
 
#define IIS2DH_ODR_MASK                 (uint8_t)0xF0
IIS2DH_Status_t IIS2DH_W_ODR(void *handle, IIS2DH_ODR_t newValue);
IIS2DH_Status_t IIS2DH_R_ODR(void *handle, IIS2DH_ODR_t *value);
 

typedef enum
{
  IIS2DH_FS_2G                          = (uint8_t)0x00,
  IIS2DH_FS_4G                          = (uint8_t)0x10,
  IIS2DH_FS_8G                          = (uint8_t)0x20,
  IIS2DH_FS_16G                         = (uint8_t)0x30
} IIS2DH_FS_t;

#define IIS2DH_FS_MASK                  (uint8_t)0x30
IIS2DH_Status_t IIS2DH_W_FS(void *handle, IIS2DH_FS_t newValue);
IIS2DH_Status_t IIS2DH_R_FS(void *handle, IIS2DH_FS_t *value);  


typedef enum
{
  IIS2DH_X_DISABLE                      = (uint8_t)0x00,
  IIS2DH_X_ENABLE                       = (uint8_t)0x01
} IIS2DH_X_t;

#define IIS2DH_X_MASK                   (uint8_t)0x01
IIS2DH_Status_t IIS2DH_W_XEN(void *handle, IIS2DH_X_t newValue);
IIS2DH_Status_t IIS2DH_R_XEN(void *handle, IIS2DH_X_t *value);


typedef enum
{
  IIS2DH_Y_DISABLE                      = (uint8_t)0x00,
  IIS2DH_Y_ENABLE                       = (uint8_t)0x02
} IIS2DH_Y_t;

#define IIS2DH_Y_MASK                   (uint8_t)0x02
IIS2DH_Status_t IIS2DH_W_YEN(void *handle, IIS2DH_Y_t newValue);
IIS2DH_Status_t IIS2DH_R_YEN(void *handle, IIS2DH_Y_t *value);


typedef enum
{
  IIS2DH_Z_DISABLE                      = (uint8_t)0x00,
  IIS2DH_Z_ENABLE                       = (uint8_t)0x04
} IIS2DH_Z_t;

#define IIS2DH_Z_MASK                   (uint8_t)0x04
IIS2DH_Status_t IIS2DH_W_ZEN(void *handle, IIS2DH_Z_t newValue);
IIS2DH_Status_t IIS2DH_R_ZEN(void *handle, IIS2DH_Z_t *value);


typedef enum
{
  IIS2DH_X_ACTIVE                       = (uint8_t)0x01,
  IIS2DH_Y_ACTIVE                       = (uint8_t)0x02,
  IIS2DH_Z_ACTIVE                       = (uint8_t)0x04,
  IIS2DH_ALL_ACTIVE                     = (uint8_t)0x07,
} IIS2DH_ACTIVE_AXIS_t;

#define IIS2DH_ACTIVE_AXIS_MASK         (IIS2DH_X_MASK | IIS2DH_Y_MASK | IIS2DH_Z_MASK)
IIS2DH_Status_t IIS2DH_W_Select_Axis(void *handle, IIS2DH_ACTIVE_AXIS_t newValue);
IIS2DH_Status_t IIS2DH_R_Select_Axis(void *handle, IIS2DH_ACTIVE_AXIS_t *value);


typedef enum
{
  IIS2DH_INT1_DRDY_DISABLED             = (uint8_t)0x00,
  IIS2DH_INT1_DRDY_ENABLED              = (uint8_t)0x10
} IIS2DH_INT1_DRDY_t;

#define   IIS2DH_INT1_DRDY_MASK         (uint8_t)0x10
IIS2DH_Status_t IIS2DH_W_DRDY_on_INT1(void *handle, IIS2DH_INT1_DRDY_t newValue);
IIS2DH_Status_t IIS2DH_R_DRDY_on_INT1(void *handle, IIS2DH_INT1_DRDY_t *value);


typedef enum
{
  IIS2DH_BDU_DISABLED                   = (uint8_t)0x00,
  IIS2DH_BDU_ENABLED                    = (uint8_t)0x80
} IIS2DH_BDU_t;

#define IIS2DH_BDU_MASK                 (uint8_t)0x80
IIS2DH_Status_t IIS2DH_W_BDU(void *handle, IIS2DH_BDU_t newValue);
IIS2DH_Status_t IIS2DH_R_BDU(void *handle, IIS2DH_BDU_t *value);


typedef enum
{
  IIS2DH_FDS_DISABLED                   = (uint8_t)0x00,
  IIS2DH_FDS_ENABLED                    = (uint8_t)0x08
} IIS2DH_HPF_FDS_t;

#define IIS2DH_HPF_FDS_MASK            (uint8_t)0x08
IIS2DH_Status_t IIS2DH_W_HPF_FDS(void *handle, IIS2DH_HPF_FDS_t newValue);
IIS2DH_Status_t IIS2DH_R_HPF_FDS(void *handle, IIS2DH_HPF_FDS_t *value);


typedef enum
{
  IIS2DH_HPF_MODE_RESET_BY_REF_REG      = (uint8_t)0x00,
  IIS2DH_HPF_MODE_REF_SIGNAL            = (uint8_t)0x40,
  IIS2DH_HPF_MODE_NORMAL                = (uint8_t)0x80,
  IIS2DH_HPF_MODE_AUTORESET             = (uint8_t)0xC0
} IIS2DH_HPF_MODE_t;

#define IIS2DH_HPF_MODE_MASK            (uint8_t)0xC0
IIS2DH_Status_t IIS2DH_W_HPF_Mode(void *handle, IIS2DH_HPF_MODE_t newValue);
IIS2DH_Status_t IIS2DH_R_HPF_Mode(void *handle, IIS2DH_HPF_MODE_t *value);

// for more information refer to IIS2DH datasheet 
typedef enum
{
  IIS2DH_CUTOFF_MODE1                   = (uint8_t)0x00,
  IIS2DH_CUTOFF_MODE2                   = (uint8_t)0x10,
  IIS2DH_CUTOFF_MODE3                   = (uint8_t)0x20,
  IIS2DH_CUTOFF_MODE4                   = (uint8_t)0x30
} IIS2DH_HPF_CUTOFF_t;

#define IIS2DH_HPF_CUTOFF_MASK          (uint8_t)0x30
IIS2DH_Status_t IIS2DH_W_HPF_Cutoff(void *handle, IIS2DH_HPF_CUTOFF_t newValue);
IIS2DH_Status_t IIS2DH_R_HPF_Cutoff(void *handle, IIS2DH_HPF_CUTOFF_t *value);


/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetAccData
* Permission    : RO
*******************************************************************************/

IIS2DH_Status_t IIS2DH_GetRawAccData(void *handle, uint8_t *buff);
IIS2DH_Status_t IIS2DH_GetSingleRawAccData(void *handle, uint8_t *buff, IIS2DH_ACTIVE_AXIS_t axis);
IIS2DH_Status_t IIS2DH_Clear_DRDY(void *handle, IIS2DH_ACTIVE_AXIS_t axis);

#endif
