/**
 ******************************************************************************
 * @file    IIS2DH_Driver_HL.h
 * @author  MP
 * @version V1.0.0
 * @date    30-June-2016
 * @brief   This file contains definitions for the IIS2DH_Driver_HL.c firmware 
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
#ifndef __IIS2DH_DRIVER_HL_H
#define __IIS2DH_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include "accelerometer.h"

/* Include accelero sensor component drivers. */
#include "IIS2DH_Driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup IIS2DH IIS2DH
 * @{
 */

/** @addtogroup IIS2DH_Public_Constants Public constants
 * @{
 */

/** @addtogroup IIS2DH_ACC_SENSITIVITY Accelero typical sensitivity values based on selected full scale and operating mode
 * @{
 */

#define IIS2DH_ACC_SENSITIVITY_FS_2G_LPM                        15.63
#define IIS2DH_ACC_SENSITIVITY_FS_2G_NM                         3.91
#define IIS2DH_ACC_SENSITIVITY_FS_2G_HR                         0.98
  
#define IIS2DH_ACC_SENSITIVITY_FS_4G_LPM                        31.25
#define IIS2DH_ACC_SENSITIVITY_FS_4G_NM                         7.81
#define IIS2DH_ACC_SENSITIVITY_FS_4G_HR                         1.95
  
#define IIS2DH_ACC_SENSITIVITY_FS_8G_LPM                        62.50
#define IIS2DH_ACC_SENSITIVITY_FS_8G_NM                         15.63
#define IIS2DH_ACC_SENSITIVITY_FS_8G_HR                         3.91
  
#define IIS2DH_ACC_SENSITIVITY_FS_16G_LPM                       188.68
#define IIS2DH_ACC_SENSITIVITY_FS_16G_NM                        46.95
#define IIS2DH_ACC_SENSITIVITY_FS_16G_HR                        11.72

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup IIS2DH_Public_Types IIS2DH Public Types
 * @{
 */

/**
 * @brief IIS2DH extended features driver internal structure definition
 */
  
typedef struct
{
  DrvStatusTypeDef ( *Get_AxesSuperRaw ) ( DrvContextTypeDef*, int16_t *, ACTIVE_AXIS_t );
  DrvStatusTypeDef ( *Get_OpMode )       ( DrvContextTypeDef*, OPER_MODE_t * );
  DrvStatusTypeDef ( *Set_OpMode )       ( DrvContextTypeDef*, OPER_MODE_t );
  DrvStatusTypeDef ( *Get_Active_Axis )  ( DrvContextTypeDef*, ACTIVE_AXIS_t * );
  DrvStatusTypeDef ( *Set_Active_Axis )  ( DrvContextTypeDef*, ACTIVE_AXIS_t );
  DrvStatusTypeDef ( *Enable_HP_Filter)  ( DrvContextTypeDef*, HPF_MODE_t , HPF_CUTOFF_t );
  DrvStatusTypeDef ( *Disable_HP_Filter) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *ClearDRDY)         ( DrvContextTypeDef*, ACTIVE_AXIS_t );
  DrvStatusTypeDef ( *Set_INT1_DRDY)     ( DrvContextTypeDef*, INT1_DRDY_CONFIG_t );
} IIS2DH_ExtDrv_t;

/**
 * @brief IIS2DH accelero specific data internal structure definition
 */
  
typedef struct
{
  float Previous_ODR;
  IIS2DH_OPER_MODE_t Previous_OpMode;
} IIS2DH_X_Data_t;



/** @addtogroup IIS2DH_Public_Variables Public variables
 * @{
 */

extern ACCELERO_Drv_t IIS2DH_Drv;
extern IIS2DH_ExtDrv_t IIS2DH_ExtDrv;

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __IIS2DH_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
