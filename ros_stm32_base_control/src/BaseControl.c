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
	/* Initialize peripherals */
	errorCode = mlsPeriphImuInit();

	/* Start Timer Interrupt*/
	errorCode = mlsBaseControlStartTimerInterrupt(&htim6);
#if (USE_UART_ROS == 1)
	/* Initialize ROS*/
	mlsBaseControlROSSetup();
#elif (USE_UART_MATLAB == 1)
	errorCode = mlsPeriphUartInit();
#endif
	return errorCode;
}

mlsErrorCode_t mlsBaseControlMain(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

#if (USE_UART_ROS == 1)
	/* Control motor*/
	if(gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] == 1)
	{
		mlsBaseControlUpdateGoalVel();
		gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] = 0;
	}

	/* Publish motor velocity data to topic "robot_vel"*/
	if(gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] == 1)
	{
		mlsBaseControlPublishMortorVelocityMsg();
		gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] = 0;
	}

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

#elif (USE_UART_MATLAB == 1)
	/* Publish IMU data to MATLAB*/
	if(gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] == 1)
	{
		mlsBaseControlUartPublishIMU();
		gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] = 0;
	}
#endif

	return errorCode;
}

/**@}*/
