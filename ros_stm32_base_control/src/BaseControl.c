/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file BaseControl.c
 * @brief Firmware communicates with ROS over rosserial protocol.
 *
 * Long description.
 * @date 2024-07-13
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "BaseControl.h"
#include "BaseControlPrivate.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/
uint8_t gBaseControlTimeUpdateFlag[10];
extern TIM_HandleTypeDef htim6;
/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
mlsErrorCode_t mlsBaseControlInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	/* Start Timer Interrupt*/
	errorCode = mlsBaseControlStartTimerInterrupt(&htim6);

	/* Initialize ROS*/
	mlsBaseControlROSSetup();

	return errorCode;
}

mlsErrorCode_t mlsBaseControlMain(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	/* Publish IMU data to topic "imu"*/
	if(gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] == 1)
	{
		mlsBaseControlPublishImuMsg();
		gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] = 0;
	}

	/* Send log message*/
	mlsBaseControlSendLogMsg();

	/* Spin NodeHandle to keep synchorus */
	mlsBaseControlSpinOnce();

	/* Keep rosserial connection */
	mlsBaseControlWaitSerialLink(mlsBaseControlConnectStatus());

	errorCode = MLS_SUCCESS;
	return errorCode;
}

/**@}*/
