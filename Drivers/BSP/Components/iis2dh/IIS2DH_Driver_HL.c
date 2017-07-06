/**
 ******************************************************************************
 * @file    IIS2DH_Driver_HL.c
 * @author  MP
 * @version V1.0.0
 * @date    30-June-2016
 * @brief   This file provides a set of high-level functions needed to manage
            the IIS2DH sensor
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

/* Includes ------------------------------------------------------------------*/

#include "IIS2DH_Driver_HL.h"
#include <math.h>



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup IIS2DH IIS2DH
 * @{
 */

/** @addtogroup IIS2DH_Callable_Private_Function_Prototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef IIS2DH_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef IIS2DH_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef IIS2DH_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef IIS2DH_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef IIS2DH_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef IIS2DH_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef IIS2DH_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration );
static DrvStatusTypeDef IIS2DH_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef IIS2DH_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef IIS2DH_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef IIS2DH_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef IIS2DH_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef IIS2DH_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef IIS2DH_Set_FS( DrvContextTypeDef *handle, SensorFs_t fs );
static DrvStatusTypeDef IIS2DH_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef IIS2DH_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled );
static DrvStatusTypeDef IIS2DH_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz );
static DrvStatusTypeDef IIS2DH_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef IIS2DH_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef IIS2DH_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

/**
 * @}
 */

/** @addtogroup IIS2DH_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef IIS2DH_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t *pData );
static DrvStatusTypeDef IIS2DH_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef IIS2DH_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef IIS2DH_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef IIS2DH_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );

/**
 * @}
 */


/** @addtogroup IIS2DH_Callable_Private_Function_Ext_Prototypes Callable private function for extended features prototypes
 * @{
 */

static DrvStatusTypeDef IIS2DH_Get_AxesSuperRaw( DrvContextTypeDef *handle, int16_t *pData, ACTIVE_AXIS_t axl_axis );
static DrvStatusTypeDef IIS2DH_Get_OpMode( DrvContextTypeDef *handle, OPER_MODE_t *axl_opMode );
static DrvStatusTypeDef IIS2DH_Set_OpMode( DrvContextTypeDef *handle, OPER_MODE_t axl_opMode );
static DrvStatusTypeDef IIS2DH_Get_Active_Axis( DrvContextTypeDef *handle, ACTIVE_AXIS_t *axl_axis );
static DrvStatusTypeDef IIS2DH_Set_Active_Axis( DrvContextTypeDef *handle, ACTIVE_AXIS_t axl_axis );
static DrvStatusTypeDef IIS2DH_Enable_HP_Filter( DrvContextTypeDef *handle, HPF_MODE_t axl_mode, HPF_CUTOFF_t axl_cutoff );
static DrvStatusTypeDef IIS2DH_Disable_HP_Filter( DrvContextTypeDef *handle );
static DrvStatusTypeDef IIS2DH_ClearDRDY( DrvContextTypeDef *handle, ACTIVE_AXIS_t axl_axis );
static DrvStatusTypeDef IIS2DH_Set_INT1_DRDY( DrvContextTypeDef *handle, INT1_DRDY_CONFIG_t axl_drdyStatus );

/**
 * @}
 */


/** @addtogroup IIS2DH_Public_Variables Public variables
 * @{
 */

/**
 * @brief IIS2DH extended features driver internal structure
 */
IIS2DH_ExtDrv_t IIS2DH_ExtDrv =
{
  IIS2DH_Get_AxesSuperRaw,
  IIS2DH_Get_OpMode,
  IIS2DH_Set_OpMode,
  IIS2DH_Get_Active_Axis,
  IIS2DH_Set_Active_Axis,
  IIS2DH_Enable_HP_Filter,
  IIS2DH_Disable_HP_Filter,
  IIS2DH_ClearDRDY,
  IIS2DH_Set_INT1_DRDY
};


/**
 * @brief IIS2DH accelero driver structure
 */
ACCELERO_Drv_t IIS2DH_Drv =
{
  IIS2DH_Init,
  IIS2DH_DeInit,
  IIS2DH_Sensor_Enable,
  IIS2DH_Sensor_Disable,
  IIS2DH_Get_WhoAmI,
  IIS2DH_Check_WhoAmI,
  IIS2DH_Get_Axes,
  IIS2DH_Get_AxesRaw,
  IIS2DH_Get_Sensitivity,
  IIS2DH_Get_ODR,
  IIS2DH_Set_ODR,
  IIS2DH_Set_ODR_Value,
  IIS2DH_Get_FS,
  IIS2DH_Set_FS,
  IIS2DH_Set_FS_Value,
  IIS2DH_Get_Axes_Status,
  IIS2DH_Set_Axes_Status,
  IIS2DH_Read_Reg,
  IIS2DH_Write_Reg,
  IIS2DH_Get_DRDY_Status
};


/** @addtogroup IIS2DH_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the IIS2DH accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Init( DrvContextTypeDef *handle )
{

  uint8_t axes_status[] = { 1, 1, 1 };
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  IIS2DH_X_Data_t *pComponentData = ( IIS2DH_X_Data_t * )pData->pComponentData;
  
  if ( IIS2DH_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Enable BDU */
  if ( IIS2DH_W_BDU( (void *)handle, IIS2DH_BDU_ENABLED ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* FIFO bypass mode selection */
  if ( IIS2DH_W_FIFO_MODE( (void *)handle, IIS2DH_FIFO_MODE_BYPASS ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Select default output data rate. */
  pComponentData->Previous_ODR = 100.0f;
  
  /* Output data rate selection - power down. */
  if ( IIS2DH_W_ODR( (void *)handle, IIS2DH_ODR_PD ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Select default operating mode. */
  pComponentData->Previous_OpMode = IIS2DH_NORMAL_MODE;
  
  /* Operating mode selection */
  if ( IIS2DH_W_OpMode( (void *)handle, IIS2DH_NORMAL_MODE ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Full scale selection. */
  if ( IIS2DH_Set_FS( handle, FS_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
     
  /* Enable axes. */
  if ( IIS2DH_Set_Axes_Status( handle, axes_status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  handle->isInitialized = 1;
  
  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the IIS2DH accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_DeInit( DrvContextTypeDef *handle )
{

  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  IIS2DH_X_Data_t *pComponentData = ( IIS2DH_X_Data_t * )pData->pComponentData;
  
  if ( IIS2DH_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
    
  /* Disable the component */
  if( IIS2DH_Sensor_Disable( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Reset output data rate. */
  pComponentData->Previous_ODR = 0.0f;
  
  /* Reset operating mode. */
  pComponentData->Previous_OpMode = IIS2DH_NORMAL_MODE;
    
  handle->isInitialized = 0;
  
  return COMPONENT_OK;
}



/**
 * @brief Enable the IIS2DH accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Sensor_Enable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  IIS2DH_X_Data_t *pComponentData = ( IIS2DH_X_Data_t * )pData->pComponentData;
  
  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }
  
  /* Operating mode selection */
  if ( IIS2DH_W_OpMode( (void *)handle, pComponentData->Previous_OpMode ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Output data rate selection. */
  if ( IIS2DH_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
      
  handle->isEnabled = 1;
  
  return COMPONENT_OK;
}



/**
 * @brief Disable the IIS2DH accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Sensor_Disable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  IIS2DH_X_Data_t *pComponentData = ( IIS2DH_X_Data_t * )pData->pComponentData;
  
  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }
  
  /* Store actual output data rate. */
  if ( IIS2DH_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  if ( IIS2DH_R_OpMode( (void *)handle, &( pComponentData->Previous_OpMode )) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
    
  /* Output data rate selection - power down. */
  if ( IIS2DH_W_ODR( (void *)handle, IIS2DH_ODR_PD ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  handle->isEnabled = 0;
  
  return COMPONENT_OK;
}

/**
 * @brief Get the IIS2DH accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration )
{

  int16_t dataRaw[3];
  float sensitivity = 0;
  
  /* Read raw data from IIS2DH output register. */
  if ( IIS2DH_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Get IIS2DH actual sensitivity. */
  if ( IIS2DH_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Calculate the data. */
  acceleration->AXIS_X = ( int32_t )( dataRaw[0] * sensitivity );
  acceleration->AXIS_Y = ( int32_t )( dataRaw[1] * sensitivity );
  acceleration->AXIS_Z = ( int32_t )( dataRaw[2] * sensitivity );
  
  return COMPONENT_OK;
}

/**
 * @brief Get the IIS2DH accelerometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

  int16_t dataRaw[3];
  
  /* Read raw data from IIS2DH output register. */
  if ( IIS2DH_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Set the raw data. */
  value->AXIS_X = dataRaw[0];
  value->AXIS_Y = dataRaw[1];
  value->AXIS_Z = dataRaw[2];
  
  return COMPONENT_OK;
}



/**
 * @brief Get the IIS2DH accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{

  IIS2DH_FS_t fullScale;
  IIS2DH_OPER_MODE_t opMode; 
     
  
  if ( IIS2DH_R_OpMode( (void *)handle, &opMode) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }  
  
  /* Read actual full scale selection from sensor. */
  if ( IIS2DH_R_FS( (void *)handle, &fullScale ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case IIS2DH_FS_2G:
      if( opMode == IIS2DH_LOW_PWR_MODE){
        *sensitivity = 15.63f;
      }
      if( opMode == IIS2DH_NORMAL_MODE){
        *sensitivity = 3.91f;
      }
      if( opMode == IIS2DH_HIGH_RES_MODE){
        *sensitivity = 0.98f;
      }
      break;
    case IIS2DH_FS_4G:
      if( opMode == IIS2DH_LOW_PWR_MODE){
        *sensitivity = 31.25f;
      }
      if( opMode == IIS2DH_NORMAL_MODE){
        *sensitivity = 7.81f;
      }
      if( opMode == IIS2DH_HIGH_RES_MODE){
        *sensitivity = 1.95f;
      }
      break;
    case IIS2DH_FS_8G:
      if( opMode == IIS2DH_LOW_PWR_MODE){
        *sensitivity = 62.50;
      }
      if( opMode == IIS2DH_NORMAL_MODE){
        *sensitivity = 15.63f;
      }
      if( opMode == IIS2DH_HIGH_RES_MODE){
        *sensitivity = 3.91f;
      }
      break;
    case IIS2DH_FS_16G:
      if( opMode == IIS2DH_LOW_PWR_MODE){
        *sensitivity = 188.68f;
      }
      if( opMode == IIS2DH_NORMAL_MODE){
        *sensitivity = 46.95f;
      }
      if( opMode == IIS2DH_HIGH_RES_MODE){
        *sensitivity = 11.72f;
      }
      break;
    default:
      *sensitivity = -1.0f;
      return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Get the IIS2DH accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  IIS2DH_ODR_t odr_low_level;
  IIS2DH_OPER_MODE_t opMode; 
     
  
  if ( IIS2DH_R_OpMode( (void *)handle, &opMode) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  
  if ( IIS2DH_R_ODR( (void *)handle, &odr_low_level ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
    
  switch( odr_low_level )
  {
    case IIS2DH_ODR_PD:
      *odr =    0.0f;
      break;
    case IIS2DH_ODR_1HZ:
      *odr =    1.0f;
      break;
    case IIS2DH_ODR_10HZ:
      *odr =    10.0f;
      break;
    case IIS2DH_ODR_25HZ:
      *odr =    25.0f;
      break;
    case IIS2DH_ODR_50HZ:
      *odr =    50.0f;
      break;
    case IIS2DH_ODR_100HZ:
      *odr =    100.0f;
      break;
    case IIS2DH_ODR_200HZ:
      *odr =    200.0f;
      break;
    case IIS2DH_ODR_400HZ:
      *odr =    400.0f;
      break;
    case IIS2DH_ODR_1620HZ_LP:
      if( opMode == IIS2DH_LOW_PWR_MODE){
        
        *odr =    1620.0f;
      } else {
        
        *odr =    -1.0f;
        return COMPONENT_ERROR;
      }
      break;
    case IIS2DH_ODR_5376HZ_LP_or_1344HZ_NM_HR:
      if( opMode == IIS2DH_LOW_PWR_MODE){
        *odr =  5376.0f;
      } else {
        *odr =  1344.0f;
      }
      break;
    default:
      *odr =    -1.0f;
      return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the IIS2DH accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  if(handle->isEnabled == 1)
  {
    if(IIS2DH_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(IIS2DH_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the IIS2DH accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  if(handle->isEnabled == 1)
  {
    if(IIS2DH_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(IIS2DH_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Get the IIS2DH accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{

  IIS2DH_FS_t fs_low_level;
  
  if ( IIS2DH_R_FS( (void *)handle, &fs_low_level ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  switch( fs_low_level )
  {
    case IIS2DH_FS_2G:
      *fullScale = 2.0f;
      break;
    case IIS2DH_FS_4G:
      *fullScale = 4.0f;
      break;
    case IIS2DH_FS_8G:
      *fullScale = 8.0f;
      break;
    case IIS2DH_FS_16G:
      *fullScale = 16.0f;
      break;
    default:
      *fullScale = -1.0f;
      return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the IIS2DH accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{

  IIS2DH_FS_t new_fs;
  
  switch( fullScale )
  {
    case FS_LOW:
      new_fs = IIS2DH_FS_2G;
      break;
    case FS_MID:
      new_fs = IIS2DH_FS_4G;
      break;
    case FS_HIGH:
      new_fs = IIS2DH_FS_8G;
      break;
    default:
      return COMPONENT_ERROR;
  }
  
  if ( IIS2DH_W_FS( (void *)handle, new_fs ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the IIS2DH accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{

  IIS2DH_FS_t new_fs;
  
  new_fs =   ( fullScale <= 2.0f ) ? IIS2DH_FS_2G
           : ( fullScale <= 4.0f ) ? IIS2DH_FS_4G
           : ( fullScale <= 8.0f ) ? IIS2DH_FS_8G
           :                         IIS2DH_FS_16G;
           
  if ( IIS2DH_W_FS( (void *)handle, new_fs ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Get the IIS2DH accelerometer sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled )
{

  IIS2DH_X_t xStatus;
  IIS2DH_Y_t yStatus;
  IIS2DH_Z_t zStatus;
  
  if ( IIS2DH_R_XEN( (void *)handle, &xStatus ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( IIS2DH_R_YEN( (void *)handle, &yStatus ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( IIS2DH_R_ZEN( (void *)handle, &zStatus ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  xyz_enabled[0] = ( xStatus == IIS2DH_X_ENABLE ) ? 1 : 0;
  xyz_enabled[1] = ( yStatus == IIS2DH_Y_ENABLE ) ? 1 : 0;
  xyz_enabled[2] = ( zStatus == IIS2DH_Z_ENABLE ) ? 1 : 0;
  
  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the IIS2DH accelerometer sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz )
{

  if ( IIS2DH_W_XEN( (void *)handle, ( enable_xyz[0] == 1 ) ? IIS2DH_X_ENABLE : IIS2DH_X_DISABLE ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  if ( IIS2DH_W_YEN( (void *)handle, ( enable_xyz[1] == 1 ) ? IIS2DH_Y_ENABLE : IIS2DH_Y_DISABLE ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  if ( IIS2DH_W_ZEN( (void *)handle, ( enable_xyz[2] == 1 ) ? IIS2DH_Z_ENABLE : IIS2DH_Z_DISABLE ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( IIS2DH_ReadReg( (void *)handle, reg, 1, data ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( IIS2DH_WriteReg( (void *)handle, reg, 1, &data ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get accelerometer data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  IIS2DH_XDA_t status_raw;

  if ( IIS2DH_R_XDataAvail( (void *)handle, &status_raw ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case IIS2DH_XDA_AVAILABLE:
      *status = 1;
      break;
    case IIS2DH_XDA_NOT_AVAILABLE:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @}
 */


/** @addtogroup IIS2DH_Private_Functions Private functions
 * @{
 */

/**
 * @brief Get the WHO_AM_I ID of the IIS2DH sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( IIS2DH_R_WHO_AM_I( (void *)handle, ( uint8_t* )who_am_i ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Check the WHO_AM_I ID of the IIS2DH sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;
  
  if ( IIS2DH_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( who_am_i != handle->who_am_i )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Get the IIS2DH accelerometer sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData)
{

  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};
  IIS2DH_OPER_MODE_t opMode; 
  uint8_t i;
  
  
  if ( IIS2DH_R_OpMode( (void *)handle, &opMode ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Read output registers from IIS2DH_OUTX_L to IIS2DH_OUTZ_H. */
  if ( IIS2DH_GetRawAccData( (void *)handle, ( uint8_t* )regValue ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  
  /* Format the data. */
  for( i = 0; i < 3; i++ ){
    if( opMode == IIS2DH_LOW_PWR_MODE ){
      pData[i] = ((int16_t)regValue[(i*2)+1]);
      
      /* convert the 2's complement 8 bit to 2's complement 16 bit */
      if (pData[i] & 0x0080){
        pData[i] |= 0xFF00;
      }
      
    } else if( opMode == IIS2DH_NORMAL_MODE ){
      pData[i] = ((((int16_t)regValue[(i*2)+1]) << 2) + (((int16_t)regValue[i*2]) >> 6));
      
      /* convert the 2's complement 10 bit to 2's complement 16 bit */
      if (pData[i] & 0x0200){
        pData[i] |= 0xFC00;
      }
      
    } else if( opMode == IIS2DH_HIGH_RES_MODE ){
      pData[i] = ((((int16_t)regValue[(i*2)+1]) << 4) + (((int16_t)regValue[i*2]) >> 4));
      
      /* convert the 2's complement 12 bit to 2's complement 16 bit */
      if (pData[i] & 0x0800){
        pData[i] |= 0xF000;
      }
    }
  }
   
  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DH accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  IIS2DH_ODR_t new_odr;
  
  switch( odr )
  {
    case ODR_LOW:
      new_odr = IIS2DH_ODR_1HZ;
      break;
    case ODR_MID_LOW:
      new_odr = IIS2DH_ODR_10HZ;
      break;
    case ODR_MID:
      new_odr = IIS2DH_ODR_25HZ;
      break;
    case ODR_MID_HIGH:
      new_odr = IIS2DH_ODR_50HZ;
      break;
    case ODR_HIGH:
      new_odr = IIS2DH_ODR_100HZ;
      break;
    default:
      return COMPONENT_ERROR;
  }
  
  if ( IIS2DH_W_ODR( (void *)handle, new_odr ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DH accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  IIS2DH_X_Data_t *pComponentData = ( IIS2DH_X_Data_t * )pData->pComponentData;
  
  switch( odr )
  {
    case ODR_LOW:
      pComponentData->Previous_ODR = 1.0f;
      break;
    case ODR_MID_LOW:
      pComponentData->Previous_ODR = 10.0f;
      break;
    case ODR_MID:
      pComponentData->Previous_ODR = 25.0f;
      break;
    case ODR_MID_HIGH:
      pComponentData->Previous_ODR = 50.0f;
      break;
    case ODR_HIGH:
      pComponentData->Previous_ODR = 100.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DH accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{
  IIS2DH_OPER_MODE_t opMode; 
  IIS2DH_ODR_t new_odr;
   
  if ( IIS2DH_R_OpMode( (void *)handle, &opMode ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  
  if ( opMode == IIS2DH_LOW_PWR_MODE ){
  
    new_odr =   ( odr <=    1.0f ) ? IIS2DH_ODR_1HZ
              : ( odr <=   10.0f ) ? IIS2DH_ODR_10HZ
              : ( odr <=   25.0f ) ? IIS2DH_ODR_25HZ
              : ( odr <=   50.0f ) ? IIS2DH_ODR_50HZ
              : ( odr <=  100.0f ) ? IIS2DH_ODR_100HZ
              : ( odr <=  200.0f ) ? IIS2DH_ODR_200HZ
              : ( odr <=  400.0f ) ? IIS2DH_ODR_400HZ
              : ( odr <= 1620.0f ) ? IIS2DH_ODR_1620HZ_LP
              :                      IIS2DH_ODR_5376HZ_LP_or_1344HZ_NM_HR;            
            
  } else {
  
    new_odr =   ( odr <=    1.0f ) ? IIS2DH_ODR_1HZ
              : ( odr <=   10.0f ) ? IIS2DH_ODR_10HZ
              : ( odr <=   25.0f ) ? IIS2DH_ODR_25HZ
              : ( odr <=   50.0f ) ? IIS2DH_ODR_50HZ
              : ( odr <=  100.0f ) ? IIS2DH_ODR_100HZ
              : ( odr <=  200.0f ) ? IIS2DH_ODR_200HZ
              : ( odr <=  400.0f ) ? IIS2DH_ODR_400HZ
              :                      IIS2DH_ODR_5376HZ_LP_or_1344HZ_NM_HR; 
  
  }
  
  if ( IIS2DH_W_ODR( (void *)handle, new_odr ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Set the IIS2DH accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{

  IIS2DH_OPER_MODE_t opMode; 
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  IIS2DH_X_Data_t *pComponentData = ( IIS2DH_X_Data_t * )pData->pComponentData;
       
  if ( IIS2DH_R_OpMode( (void *)handle, &opMode ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  
  if ( opMode == IIS2DH_LOW_PWR_MODE ){
  
    pComponentData->Previous_ODR =  ( odr <=    1.0f ) ? 1.0f
                                  : ( odr <=   10.0f ) ? 10.0f
                                  : ( odr <=   25.0f ) ? 25.0f
                                  : ( odr <=   50.0f ) ? 50.0f
                                  : ( odr <=  100.0f ) ? 100.0f
                                  : ( odr <=  200.0f ) ? 200.0f
                                  : ( odr <=  400.0f ) ? 400.0f
                                  : ( odr <= 1620.0f ) ? 1620.0f
                                  :                      5376.0f;       
            
  } else {
  
    pComponentData->Previous_ODR =  ( odr <=    1.0f ) ? 1.0f
                                  : ( odr <=   10.0f ) ? 10.0f
                                  : ( odr <=   25.0f ) ? 25.0f
                                  : ( odr <=   50.0f ) ? 50.0f
                                  : ( odr <=  100.0f ) ? 100.0f
                                  : ( odr <=  200.0f ) ? 200.0f
                                  : ( odr <=  400.0f ) ? 400.0f
                                  :                      1344.0f; 
  
  }
  
  return COMPONENT_OK;
}


/** @addtogroup IIS2DH_Callable_Private_Functions_Ext Callable private functions for extended features
 * @{
 */

/**
 * @brief Get the IIS2DH accelerometer sensor super raw axes
 * @param handle the device handle
 * @param pData pointer where the super raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_AxesSuperRaw( DrvContextTypeDef *handle, int16_t *pData, ACTIVE_AXIS_t axl_axis )
{  
  uint8_t regValue[2] = {0, 0};
  IIS2DH_ACTIVE_AXIS_t axis;
  
  switch( axl_axis ){
  case X_AXIS:
    axis = IIS2DH_X_ACTIVE;
    break;
  case Y_AXIS:
    axis = IIS2DH_Y_ACTIVE;
    break;
  case Z_AXIS:
    axis = IIS2DH_Z_ACTIVE;
    break;
  case ALL_ACTIVE:
    axis = IIS2DH_ALL_ACTIVE;
    break;
  default:
    return COMPONENT_ERROR;    
  }
  
  if ( IIS2DH_GetSingleRawAccData( (void *)handle, regValue, axis ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  *pData = ((((int16_t)regValue[1]) << 8) + ((int16_t)regValue[0]));
  
  return COMPONENT_OK;
  
}

/**
 * @brief Get the Operating Mode of the IIS2DH sensor
 * @param handle the device handle
 * @param opMode pointer to the value of Operating mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_OpMode( DrvContextTypeDef *handle, OPER_MODE_t *axl_opMode )
{  
  IIS2DH_OPER_MODE_t opMode;
    
  if ( IIS2DH_R_OpMode( (void *)handle, &opMode ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  switch( opMode ){
  case IIS2DH_NORMAL_MODE:
    *axl_opMode = NORMAL_MODE;
    break;
  case IIS2DH_HIGH_RES_MODE:
    *axl_opMode = HIGH_RES_MODE;
    break;
  case IIS2DH_LOW_PWR_MODE:
    *axl_opMode = LOW_PWR_MODE;
    break;
  default:
    return COMPONENT_ERROR;    
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Set the Operating Mode of the IIS2DH sensor
 * @param handle the device handle
 * @param opMode the operating mode to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_OpMode( DrvContextTypeDef *handle, OPER_MODE_t axl_opMode )
{  
  IIS2DH_OPER_MODE_t opMode;
  
  switch( axl_opMode ){
  case NORMAL_MODE:
    opMode = IIS2DH_NORMAL_MODE;
    break;
  case HIGH_RES_MODE:
    opMode = IIS2DH_HIGH_RES_MODE;
    break;
  case LOW_PWR_MODE:
    opMode = IIS2DH_LOW_PWR_MODE;
    break;
  default:
    return COMPONENT_ERROR;    
  }
  
  if ( IIS2DH_W_OpMode( (void *)handle, opMode ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Get the active axis of the IIS2DH sensor
 * @param handle the device handle
 * @param axis pointer to the value of active axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Get_Active_Axis( DrvContextTypeDef *handle, ACTIVE_AXIS_t *axl_axis )
{  
  IIS2DH_ACTIVE_AXIS_t axis;
   
  if ( IIS2DH_R_Select_Axis( (void *)handle, &axis ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  switch( axis ){
  case IIS2DH_X_ACTIVE:
    *axl_axis = X_AXIS;
    break;
  case IIS2DH_Y_ACTIVE:
    *axl_axis = Y_AXIS;
    break;
  case IIS2DH_Z_ACTIVE:
    *axl_axis = Z_AXIS;
    break;
  case IIS2DH_ALL_ACTIVE:
    *axl_axis = ALL_ACTIVE;
    break;
  default:
    return COMPONENT_ERROR;    
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Set the active axis of the IIS2DH sensor
 * @param handle the device handle
 * @param axis the active axis to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_Active_Axis( DrvContextTypeDef *handle, ACTIVE_AXIS_t axl_axis )
{  
  IIS2DH_ACTIVE_AXIS_t axis;
  
  switch( axl_axis ){
  case X_AXIS:
    axis = IIS2DH_X_ACTIVE;
    break;
  case Y_AXIS:
    axis = IIS2DH_Y_ACTIVE;
    break;
  case Z_AXIS:
    axis = IIS2DH_Z_ACTIVE;
    break;
  case ALL_ACTIVE:
    axis = IIS2DH_ALL_ACTIVE;
    break;
  default:
    return COMPONENT_ERROR;    
  }
  
  if ( IIS2DH_W_Select_Axis( (void *)handle, axis ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Enable the HP Filter of the IIS2DH sensor
 * @param handle the device handle
 * @param mode the HP Filter mode to be set
 * @param cutoff the HP Filter cutoff frequency to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Enable_HP_Filter( DrvContextTypeDef *handle, HPF_MODE_t axl_mode, HPF_CUTOFF_t axl_cutoff )
{     
  IIS2DH_HPF_MODE_t mode;
  IIS2DH_HPF_CUTOFF_t cutoff;
    
  switch( axl_mode ){
  case HPF_MODE_RESET_BY_REF_REG:
    mode = IIS2DH_HPF_MODE_RESET_BY_REF_REG;
    break;
  case HPF_MODE_REF_SIGNAL:
    mode = IIS2DH_HPF_MODE_REF_SIGNAL;
    break;
  case HPF_MODE_NORMAL:
    mode = IIS2DH_HPF_MODE_NORMAL;
    break;
  case HPF_MODE_AUTORESET:
    mode = IIS2DH_HPF_MODE_AUTORESET;
    break;
  default:
    return COMPONENT_ERROR;    
  }
  
  switch( axl_cutoff ){
  case CUTOFF_MODE1:
    cutoff = IIS2DH_CUTOFF_MODE1;
    break;
  case CUTOFF_MODE2:
    cutoff = IIS2DH_CUTOFF_MODE2;
    break;
  case CUTOFF_MODE3:
    cutoff = IIS2DH_CUTOFF_MODE3;
    break;
  case CUTOFF_MODE4:
    cutoff = IIS2DH_CUTOFF_MODE4;
    break;
  default:
    return COMPONENT_ERROR;    
  }
      
  if ( IIS2DH_W_HPF_Mode( (void *)handle, mode ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  if ( IIS2DH_W_HPF_Cutoff( (void *)handle, cutoff ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  if ( IIS2DH_W_HPF_FDS( (void *)handle, IIS2DH_FDS_ENABLED ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Disable the HP Filter of the IIS2DH sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Disable_HP_Filter( DrvContextTypeDef *handle )
{      
  if ( IIS2DH_W_HPF_FDS( (void *)handle, IIS2DH_FDS_DISABLED ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Clear DRDY of the IIS2DH sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_ClearDRDY( DrvContextTypeDef *handle, ACTIVE_AXIS_t axl_axis )
{ 
  IIS2DH_ACTIVE_AXIS_t axis;
  
  switch( axl_axis ){
  case X_AXIS:
    axis = IIS2DH_X_ACTIVE;
    break;
  case Y_AXIS:
    axis = IIS2DH_Y_ACTIVE;
    break;
  case Z_AXIS:
    axis = IIS2DH_Z_ACTIVE;
    break;
  case ALL_ACTIVE:
    axis = IIS2DH_ALL_ACTIVE;
    break;
  default:
    return COMPONENT_ERROR;    
  }
  
  if (IIS2DH_Clear_DRDY( (void *)handle, axis ) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }   
  
  return COMPONENT_OK;
}

  

/**
 * @brief Set DRDY enable/disable of the IIS2DH sensor on INT1 
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef IIS2DH_Set_INT1_DRDY( DrvContextTypeDef *handle, INT1_DRDY_CONFIG_t axl_drdyStatus )
{  
  IIS2DH_INT1_DRDY_t drdyStatus;
  
  switch( axl_drdyStatus ){
  case INT1_DRDY_DISABLED:
    drdyStatus = IIS2DH_INT1_DRDY_DISABLED;
    break;
  case INT1_DRDY_ENABLED:
    drdyStatus = IIS2DH_INT1_DRDY_ENABLED;
    break;
  default:
    return COMPONENT_ERROR;    
  }
  
  if ( IIS2DH_W_DRDY_on_INT1((void *)handle, drdyStatus) == IIS2DH_ERROR )
  {
    return COMPONENT_ERROR;
  }
      
  return COMPONENT_OK;
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
