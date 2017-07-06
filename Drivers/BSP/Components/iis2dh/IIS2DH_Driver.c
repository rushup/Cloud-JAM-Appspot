/**
 ******************************************************************************
 * @file    IIS2DH_Driver.c
 * @author  MP
 * @version V1.0.0
 * @date    30-June-2016
 * @brief   IIS2DH Platform Independent Driver
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
#include "IIS2DH_Driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Imported function prototypes ----------------------------------------------*/
extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);


/*******************************************************************************
* Function Name : IIS2DH_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*                 I2C or SPI reading functions
* Input         : Register Address
* Output        : Data Read
* Return        : None
*******************************************************************************/
IIS2DH_Status_t IIS2DH_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{

  if ( NumByteToRead > 1 ) RegAddr |= 0x80;
  
  if ( Sensor_IO_Read( handle, RegAddr, Data, NumByteToRead ) )
    return IIS2DH_ERROR;
  else
    return IIS2DH_OK;
}

/*******************************************************************************
* Function Name : IIS2DH_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*                 I2C or SPI writing function
* Input         : Register Address, Data to be written
* Output        : None
* Return        : None
*******************************************************************************/
IIS2DH_Status_t IIS2DH_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
{

  if ( NumByteToWrite > 1 ) RegAddr |= 0x80;
  
  if ( Sensor_IO_Write( handle, RegAddr, Data, NumByteToWrite ) )
    return IIS2DH_ERROR;
  else
    return IIS2DH_OK;
}


/*******************************************************************************
* Function Name  : IIS2DH_W_DRDY_on_INT1
* Description    : Write INT1_DRDY
* Input          : IIS2DH_INT1_DRDY_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t  IIS2DH_W_DRDY_on_INT1(void *handle, IIS2DH_INT1_DRDY_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG3, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_INT1_DRDY_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG3, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_DRDY_on_INT1
* Description    : Read INT1_DRDY
* Input          : Pointer to IIS2DH_INT1_DRDY_t
* Output         : Status of INT1_DRDY see IIS2DH_INT1_DRDY_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_DRDY_on_INT1(void *handle, IIS2DH_INT1_DRDY_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG3, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_INT1_DRDY_MASK; //mask

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_W_FIFO_MODE
* Description    : Write FIFO_MODE
* Input          : IIS2DH_FIFO_MODE_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_FIFO_MODE(void *handle, IIS2DH_FIFO_MODE_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_FIFO_CTRL_REG, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_FIFO_MODE_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_FIFO_CTRL_REG, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_FIFO_MODE
* Description    : Read FIFO_MODE
* Input          : Pointer to IIS2DH_FIFO_MODE_t
* Output         : Status of FIFO_MODE see IIS2DH_FIFO_MODE_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_FIFO_MODE(void *handle, IIS2DH_FIFO_MODE_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_FIFO_CTRL_REG, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_FIFO_MODE_MASK; //mask

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_WHO_AM_I
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to uint8_t
* Output         : Status of WHO_AM_I_BIT
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_WHO_AM_I(void *handle, uint8_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_WHO_AM_I_REG, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;
  
  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_W_FS
* Description    : Write FS
* Input          : IIS2DH_FS_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_FS(void *handle, IIS2DH_FS_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG4, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_FS_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG4, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_FS
* Description    : Read FS
* Input          : Pointer to IIS2DH_FS_t
* Output         : Status of FS see IIS2DH_FS_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_FS(void *handle, IIS2DH_FS_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG4, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_FS_MASK; //mask

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_XDataAvail
* Description    : Read XDA
* Input          : Pointer to IIS2DH_XDA_t
* Output         : Status of XDA see IIS2DH_XDA_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_R_XDataAvail(void *handle, IIS2DH_XDA_t *value)
{
  if( !IIS2DH_ReadReg( handle, IIS2DH_STATUS_REG, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_XDA_MASK; //mask

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_W_ODR
* Description    : Write ODR
* Input          : IIS2DH_ODR_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_ODR(void *handle, IIS2DH_ODR_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_ODR_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_ODR
* Description    : Read ODR
* Input          : Pointer to IIS2DH_ODR_t
* Output         : Status of ODR see IIS2DH_ODR_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_ODR(void *handle, IIS2DH_ODR_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_ODR_MASK; //mask

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_W_BDU
* Description    : Write BDU
* Input          : IIS2DH_BDU_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_BDU(void *handle, IIS2DH_BDU_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG4, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_BDU_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG4, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_BDU
* Description    : Read BDU
* Input          : Pointer to IIS2DH_BDU_t
* Output         : Status of BDU see IIS2DH_BDU_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_BDU(void *handle, IIS2DH_BDU_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG4, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_BDU_MASK; //mask

  return IIS2DH_OK;
}


/*******************************************************************************
* Function Name  : IIS2DH_W_Select_Axis
* Description    : Enable one axis
* Input          : IIS2DH_ACTIVE_AXIS_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_Select_Axis(void *handle, IIS2DH_ACTIVE_AXIS_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_ACTIVE_AXIS_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_Select_Axis
* Description    : Read enabled one axis
* Input          : IIS2DH_ACTIVE_AXIS_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_R_Select_Axis(void *handle, IIS2DH_ACTIVE_AXIS_t *value)
{
  
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_ACTIVE_AXIS_MASK;
  
  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_W_XEN
* Description    : Write XEN
* Input          : IIS2DH_XEN_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_XEN(void *handle, IIS2DH_X_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_X_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_XEN
* Description    : Read XEN
* Input          : Pointer to IIS2DH_XEN_t
* Output         : Status of XEN see IIS2DH_XEN_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_XEN(void *handle, IIS2DH_X_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_X_MASK; //mask

  return IIS2DH_OK;
}
/*******************************************************************************
* Function Name  : IIS2DH_W_YEN
* Description    : Write YEN
* Input          : IIS2DH_YEN_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_YEN(void *handle, IIS2DH_Y_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_Y_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_YEN
* Description    : Read YEN
* Input          : Pointer to IIS2DH_YEN_t
* Output         : Status of YEN see IIS2DH_YEN_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_YEN(void *handle, IIS2DH_Y_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_Y_MASK; //mask

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_W_ZEN
* Description    : Write ZEN
* Input          : IIS2DH_ZEN_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_ZEN(void *handle, IIS2DH_Z_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_Z_MASK;
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_ZEN
* Description    : Read ZEN
* Input          : Pointer to IIS2DH_ZEN_t
* Output         : Status of ZEN see IIS2DH_ZEN_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_ZEN(void *handle, IIS2DH_Z_t *value)
{
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;

  *value &= IIS2DH_Z_MASK; //mask

  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_GetRawAccData
* Description    : Read output register
* Input          : pointer to [uint8_t]
* Output         : GetAccData buffer uint8_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_GetRawAccData(void *handle, uint8_t *buff)
{
  
  if( IIS2DH_ReadReg(handle, IIS2DH_OUT_X_L, 6, buff))
    return IIS2DH_ERROR;
    
  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_GetSingleRawAccData
* Description    : Read output register
* Input          : pointer to [uint8_t]
*                  axis to read
* Output         : GetAccData buffer uint8_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_GetSingleRawAccData(void *handle, uint8_t *buff, IIS2DH_ACTIVE_AXIS_t axis)
{
  
  if(axis == IIS2DH_X_ACTIVE){
    if( IIS2DH_ReadReg(handle, IIS2DH_OUT_X_L, 2, buff))
      return IIS2DH_ERROR;
    
  } else if(axis == IIS2DH_Y_ACTIVE){
    if( IIS2DH_ReadReg(handle, IIS2DH_OUT_Y_L, 2, buff))
      return IIS2DH_ERROR;
    
  } else if(axis == IIS2DH_Z_ACTIVE){
    if( IIS2DH_ReadReg(handle, IIS2DH_OUT_Z_L, 2, buff))
      return IIS2DH_ERROR;
    
  }
  
  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_R_OpMode
* Description    : Get Operating mode
* Input          : Pointer to IIS2DH_OPER_MODE_t
* Output         : Status of Operating mode see IIS2DH_OPER_MODE_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/

IIS2DH_Status_t IIS2DH_R_OpMode(void *handle, IIS2DH_OPER_MODE_t *value)
{
  uint8_t tempReg = 0x00;
  uint8_t tmp; 
  
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, &tempReg) )
    return IIS2DH_ERROR;

  tmp = (tempReg & IIS2DH_OPER_MODE_MASK) >> 2;
  
  
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG4, 1, &tempReg) )
    return IIS2DH_ERROR;
  
  tmp |= (tempReg & IIS2DH_OPER_MODE_MASK) >> 3;

  *value = (IIS2DH_OPER_MODE_t)tmp;
  
  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_W_OpMode
* Description    : Set Operating mode
* Input          : IIS2DH_OPER_MODE_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_OpMode(void *handle, IIS2DH_OPER_MODE_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_OPER_MODE_MASK;
  value |= ((newValue << 2) & IIS2DH_OPER_MODE_MASK);

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG1, 1, &value) )
    return IIS2DH_ERROR;

  
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG4, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~IIS2DH_OPER_MODE_MASK;
  value |= ((newValue << 3) & IIS2DH_OPER_MODE_MASK);

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG4, 1, &value) )
    return IIS2DH_ERROR;
 
  
  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_W_HPF_Mode
* Description    : Write HP Filter mode
* Input          : IIS2DH_HPF_MODE_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_HPF_Mode(void *handle, IIS2DH_HPF_MODE_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG2, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~(IIS2DH_HPF_MODE_MASK);
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG2, 1, &value) )
    return IIS2DH_ERROR;
  
  return IIS2DH_OK;
}


/*******************************************************************************
* Function Name  : IIS2DH_R_HPF_Mode
* Description    : Read HP Filter mode
* Input          : IIS2DH_HPF_MODE_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_R_HPF_Mode(void *handle, IIS2DH_HPF_MODE_t *value)
{
  
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG2, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;
  
  *value &= IIS2DH_HPF_MODE_MASK;
  
  return IIS2DH_OK;
}


/*******************************************************************************
* Function Name  : IIS2DH_W_HPF_Cutoff
* Description    : Write HP Filter cutoff frequency
* Input          : IIS2DH_HPF_CUTOFF_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_HPF_Cutoff(void *handle, IIS2DH_HPF_CUTOFF_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG2, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~(IIS2DH_HPF_CUTOFF_MASK);
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG2, 1, &value) )
    return IIS2DH_ERROR;
  
  return IIS2DH_OK;
}


/*******************************************************************************
* Function Name  : IIS2DH_R_HPF_Cutoff
* Description    : Read HP Filter cutoff frequency
* Input          : IIS2DH_HPF_CUTOFF_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_R_HPF_Cutoff(void *handle, IIS2DH_HPF_CUTOFF_t *value)
{
  
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG2, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;
  
  *value &= IIS2DH_HPF_CUTOFF_MASK;
  
  return IIS2DH_OK;
}


/*******************************************************************************
* Function Name  : IIS2DH_W_HPF_FDS
* Description    : Write HP Filter filtered data selection
* Input          : IIS2DH_HPF_FDS_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_W_HPF_FDS(void *handle, IIS2DH_HPF_FDS_t newValue)
{
  uint8_t value;

  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG2, 1, &value) )
    return IIS2DH_ERROR;

  value &= ~(IIS2DH_HPF_FDS_MASK);
  value |= newValue;

  if( IIS2DH_WriteReg(handle, IIS2DH_CTRL_REG2, 1, &value) )
    return IIS2DH_ERROR;
  
  return IIS2DH_OK;
}


/*******************************************************************************
* Function Name  : IIS2DH_R_HPF_FDS
* Description    : Read HP Filter filtered data selection
* Input          : IIS2DH_HPF_FDS_t
* Output         : None
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_R_HPF_FDS(void *handle, IIS2DH_HPF_FDS_t *value)
{
  
  if( IIS2DH_ReadReg(handle, IIS2DH_CTRL_REG2, 1, (uint8_t *)value) )
    return IIS2DH_ERROR;
  
  *value &= IIS2DH_HPF_FDS_MASK;
  
  return IIS2DH_OK;
}

/*******************************************************************************
* Function Name  : IIS2DH_Clear_DRDY
* Description    : Read one output register
* Input          : pointer to [uint8_t]
*                  axis to read
* Output         : GetAccData buffer uint8_t
* Return         : Status [IIS2DH_ERROR, IIS2DH_OK]
*******************************************************************************/
IIS2DH_Status_t IIS2DH_Clear_DRDY(void *handle, IIS2DH_ACTIVE_AXIS_t axis)
{
  uint8_t buff;
  
  if(axis == IIS2DH_Z_ACTIVE){
    if( IIS2DH_ReadReg(handle, IIS2DH_OUT_Z_H, 1, &buff))
      return IIS2DH_ERROR;
    
  } else if(axis == IIS2DH_Y_ACTIVE){
    if( IIS2DH_ReadReg(handle, IIS2DH_OUT_Y_H, 1, &buff))
      return IIS2DH_ERROR;
    
  } else {
    if( IIS2DH_ReadReg(handle, IIS2DH_OUT_X_H, 1, &buff))
      return IIS2DH_ERROR;
    
  }
  
  return IIS2DH_OK;
}


