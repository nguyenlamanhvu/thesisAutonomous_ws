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
#include "HardwareInfo.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/
extern uint8_t gBaseControlTimeUpdateFlag[10];
/********** Local (static) variable definition ********************************/
uint32_t gBaseControlTimeUpdate[10];
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

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
/**@}*/
