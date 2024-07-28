/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file <fileName>.c
 * @brief <Brief description>
 *
 * Long description.
 * @date 2024-07-13
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "HardwareInfo.h"
#include "mpu9250.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/
#define HW_IMU_I2C				hi2c1
/********** Global variable definition section ********************************/
extern uint8_t gBaseControlTimeUpdateFlag[10];
/********** Local (static) variable definition ********************************/
uint32_t gBaseControlTimeUpdate[10];
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6)
  {
    if(gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX] >= 1000/IMU_PUBLISH_FREQUENCY)
    {
    	gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] = 1;
    	gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX] = 0;
    }

    if(gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX] >= 1000/IMU_UPDATE_FREQUENCY)
	{
    	gBaseControlTimeUpdateFlag[IMU_UPDATE_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[IMU_UPDATE_TIME_INDEX] = 0;
	}

    if(gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX] >= 1000/CONTROL_MOTOR_FREQUENCY)
    {
    	gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX] = 0;
    }

    if(gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX] >= 1000/VEL_PUBLISH_FREQUENCY)
	{
		gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX] = 0;
	}

    if(gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX] >= 1000/DRIVE_INFORMATION_FREQUENCY)
	{
		gBaseControlTimeUpdateFlag[DRIVE_INFORMATION_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX] = 0;
	}

    gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX]++;
	gBaseControlTimeUpdate[IMU_UPDATE_TIME_INDEX]++;
	gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX]++;
	gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX]++;
	gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX]++;
  }
}

/********** Global function definition section ********************************/
void mlsHardwareInfoDelay(uint32_t timeMs)
{
	HAL_Delay(timeMs);
}

mlsErrorCode_t mlsHardwareInfoStartTimerInterrupt(TIM_HandleTypeDef* timBaseHandle)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = (mlsErrorCode_t)HAL_TIM_Base_Start_IT(timBaseHandle);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoMpu9250ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	uint8_t tmpBuffer[1];
	mlsErrorCode_t errorCode = MLS_ERROR;

	tmpBuffer[0] = regAddr;
	errorCode = HAL_I2C_Master_Transmit(&HW_IMU_I2C, MPU9250_ADDR, tmpBuffer, 1, 100);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	errorCode = HAL_I2C_Master_Receive(&HW_IMU_I2C, MPU9250_ADDR, buffer, len, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoMpu9250WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	uint8_t bufferSend[len+1];

	bufferSend[0] = regAddr;
	for(uint8_t i = 0; i < len; i++)
	{
		bufferSend[i + 1] = buffer[i];
	}
	errorCode = HAL_I2C_Master_Transmit(&HW_IMU_I2C, MPU9250_ADDR, bufferSend, len + 1, 100);
	return errorCode;
}
/**@}*/
