/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file BaseControlPrivate.cpp
 * @brief Private firmware communicates with ROS over rosserial protocol.
 *
 * Long description.
 * @date 2024-07-11
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "stdio.h"
#include "stdbool.h"

#include "ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include "HardwareInfo.h"
#include "BaseControlPrivate.h"
/********** Local Constant and compile switch definition section **************/
#define USE_ROS_LOG_DEBUG
//#define USE_ROS_LOG_REPEAT_CONNECTED_DEBUG
#define PUBLISH_MAG_DEBUG

#define ROS_TOPIC_IMU                       "imu"
#define ROS_TOPIC_MAG						"mag"
#define ROS_TOPIC_VEL						"robot_vel"
#define ROS_TOPIC_CALLBACK_VEL				"callback_robot_vel"
/********** Local Type definition section *************************************/
#if (USE_UART_MATLAB == 1)
typedef union {
    float floatValue;
    uint8_t byteArray[4];
}FloatByteArray;
#endif
/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/

/********** Local (static) function declaration section ***********************/
static sensor_msgs::Imu BaseControlGetIMU(void);
static sensor_msgs::MagneticField BaseControlGetMag(void);
static ros::Time BaseControlGetROSTime(void);
static void BaseControlCallbackCommandVelocity(const geometry_msgs::Twist &callbackVelMsg);

/********** Local (static) variable definition section ************************/
ros::NodeHandle rosNodeHandle;    	/*!< ROS node handle */
char rosLogBuffer[100];          	/*!< ROS log message buffer */

char imuFrameId[20];

sensor_msgs::Imu imuMsg;           	/*!< ROS IMU message */
sensor_msgs::MagneticField magMsg;	/*!< ROS Magnetic message*/
geometry_msgs::Twist velocityMsg;	/*!< ROS velocity message*/
nav_msgs::Odometry odometryMsg;		/*!< ROS odometry message*/

ros::Subscriber<geometry_msgs::Twist> callbackVelSub(ROS_TOPIC_CALLBACK_VEL, BaseControlCallbackCommandVelocity);

ros::Publisher imuPub(ROS_TOPIC_IMU, &imuMsg);
ros::Publisher magPub(ROS_TOPIC_MAG, &magMsg);
ros::Publisher velPub(ROS_TOPIC_VEL, &velocityMsg);

float goalVelocity[2] = {0.0, 0.0};            	/*!< Velocity to control motor */
float goalReceiveVelocity[2] = {0.0, 0.0};   	/*!< Velocity receive from "callback_robot_vel" topic */

#if (USE_UART_MATLAB == 1)
FloatByteArray uartData;
uint8_t DataToSend[36];
uint8_t i;
#endif
/********** Local function definition section *********************************/
static sensor_msgs::Imu BaseControlGetIMU(void)
{
	float accelX, accelY, accelZ;
	float gyroX, gyroY, gyroZ;
	float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

	mlsPeriphImuGetAccel(&accelX, &accelY, &accelZ);
	mlsPeriphImuGetGyro(&gyroX, &gyroY, &gyroZ);

	sensor_msgs::Imu imuMsg_;

	imuMsg_.angular_velocity.x = gyroX;
	imuMsg_.angular_velocity.y = gyroY;
	imuMsg_.angular_velocity.z = gyroZ;

	imuMsg_.linear_acceleration.x = accelX;
	imuMsg_.linear_acceleration.y = accelY;
	imuMsg_.linear_acceleration.z = accelZ;

	imuMsg_.orientation.x = q0;
	imuMsg_.orientation.y = q1;
	imuMsg_.orientation.z = q2;
	imuMsg_.orientation.w = q3;

	imuMsg_.angular_velocity_covariance[0] = 0;
	imuMsg_.angular_velocity_covariance[1] = 0;
	imuMsg_.angular_velocity_covariance[2] = 0;
	imuMsg_.angular_velocity_covariance[3] = 0;
	imuMsg_.angular_velocity_covariance[4] = 0;
	imuMsg_.angular_velocity_covariance[5] = 0;
	imuMsg_.angular_velocity_covariance[6] = 0;
	imuMsg_.angular_velocity_covariance[7] = 0;
	imuMsg_.angular_velocity_covariance[8] = 0;

	imuMsg_.linear_acceleration_covariance[0] = 0;
	imuMsg_.linear_acceleration_covariance[1] = 0;
	imuMsg_.linear_acceleration_covariance[2] = 0;
	imuMsg_.linear_acceleration_covariance[3] = 0;
	imuMsg_.linear_acceleration_covariance[4] = 0;
	imuMsg_.linear_acceleration_covariance[5] = 0;
	imuMsg_.linear_acceleration_covariance[6] = 0;
	imuMsg_.linear_acceleration_covariance[7] = 0;
	imuMsg_.linear_acceleration_covariance[8] = 0;

	imuMsg_.orientation_covariance[0] = 0;
	imuMsg_.orientation_covariance[1] = 0;
	imuMsg_.orientation_covariance[2] = 0;
	imuMsg_.orientation_covariance[3] = 0;
	imuMsg_.orientation_covariance[4] = 0;
	imuMsg_.orientation_covariance[5] = 0;
	imuMsg_.orientation_covariance[6] = 0;
	imuMsg_.orientation_covariance[7] = 0;
	imuMsg_.orientation_covariance[8] = 0;

	return imuMsg_;
}

static sensor_msgs::MagneticField BaseControlGetMag(void)
{
	float magX, magY, magZ;

	mlsPeriphImuGetMag(&magX, &magY, &magZ);

	sensor_msgs::MagneticField magMsg_;

	magMsg_.magnetic_field.x = magX;
	magMsg_.magnetic_field.y = magY;
	magMsg_.magnetic_field.z = magZ;

	magMsg_.magnetic_field_covariance[0] = 0;
	magMsg_.magnetic_field_covariance[1] = 0;
	magMsg_.magnetic_field_covariance[2] = 0;
	magMsg_.magnetic_field_covariance[3] = 0;
	magMsg_.magnetic_field_covariance[4] = 0;
	magMsg_.magnetic_field_covariance[5] = 0;
	magMsg_.magnetic_field_covariance[6] = 0;
	magMsg_.magnetic_field_covariance[7] = 0;
	magMsg_.magnetic_field_covariance[8] = 0;

	return magMsg_;
}

static void BaseControlCallbackCommandVelocity(const geometry_msgs::Twist &callbackVelMsg)
{
	/* Get goal velocity */
	goalReceiveVelocity[LINEAR] = callbackVelMsg.linear.x;
	goalReceiveVelocity[ANGULAR] = callbackVelMsg.angular.z;

	/* Constrain velocity */


}

static ros::Time BaseControlGetROSTime(void)
{
	return rosNodeHandle.now();
}
/********** Global function definition section ********************************/
void mlsBaseControlROSSetup(void)
{
    rosNodeHandle.initNode(); /*!< Init ROS node handle */

    rosNodeHandle.subscribe(callbackVelSub);	/*!< Subscribe to "callback_robot_vel" topic to get control robot velocity*/

    rosNodeHandle.advertise(imuPub);	/*!< Register the publisher to "imu" topic */
    rosNodeHandle.advertise(magPub);	/*!< Register the publisher to "mag" topic */
    rosNodeHandle.advertise(velPub);	/*!< Register the publisher to "robot_vel" topic */
}

void mlsBaseControlSpinOnce(void)
{
    rosNodeHandle.spinOnce();
}

void mlsBaseControlWaitSerialLink(bool isConnected)
{
	static bool waitFlag = false;

	if (isConnected)
	{
		if (waitFlag == false)
		{
			mlsHardwareInfoDelay(10);

			waitFlag = true;
		}
	}
	else
	{
		waitFlag = false;
	}
}

bool mlsBaseControlConnectStatus(void)
{
	return rosNodeHandle.connected();
}

void mlsBaseControlSendLogMsg(void)
{
	static bool logFlag = false;

	if (rosNodeHandle.connected())
	{
		if (logFlag == false)
		{
			sprintf(rosLogBuffer, "--------------------------");
			rosNodeHandle.loginfo(rosLogBuffer);

			sprintf(rosLogBuffer, "Connected to openSTM32-Board");
			rosNodeHandle.loginfo(rosLogBuffer);

			sprintf(rosLogBuffer, "--------------------------");
			rosNodeHandle.loginfo(rosLogBuffer);

#ifdef USE_ROS_LOG_REPEAT_CONNECTED_DEBUG
			logFlag = false;
#else
			logFlag = true;
#endif
		}
	}
	else
	{
		logFlag = false;
	}
}

void mlsBaseControlPublishImuMsg(void)
{
	/* Get IMU data*/
	imuMsg = BaseControlGetIMU();

	imuMsg.header.stamp = BaseControlGetROSTime();
//	imuMsg.header.frame_id = imuFrameId;

	/* Publish IMU message*/
	imuPub.publish(&imuMsg);

#ifdef PUBLISH_MAG_DEBUG
	/* Get Mag data*/
	magMsg = BaseControlGetMag();
	magMsg.header.stamp = BaseControlGetROSTime();

	/* Publish Mag message*/
	magPub.publish(&magMsg);
#endif
}

void mlsBaseControlPublishMortorVelocityMsg(void)
{
	/* Get motor velocity */
	velocityMsg.linear.x = goalVelocity[LINEAR];
	velocityMsg.angular.z = goalVelocity[ANGULAR];

	/* Publish motor velocity message to "robot_vel" topic*/
	velPub.publish(&velocityMsg);
}

void mlsBaseControlPublishDriveInformationMsg(void)
{
	odometryMsg.header.stamp = BaseControlGetROSTime();
//	odometryMsg.header.frame_id = odomFrameId;
}

void mlsBaseControlUpdateGoalVel(void)
{
	goalVelocity[LINEAR] = goalReceiveVelocity[LINEAR];
	goalVelocity[ANGULAR] = goalReceiveVelocity[ANGULAR];
}

mlsErrorCode_t mlsBaseControlStartTimerInterrupt(TIM_HandleTypeDef* timBaseHandle)
{
	return mlsHardwareInfoStartTimerInterrupt(timBaseHandle);
}

#if (USE_UART_MATLAB == 1)
mlsErrorCode_t mlsBaseControlUartPublishIMU(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	float accelX, accelY, accelZ;
	float gyroX, gyroY, gyroZ;
	float magX, magY, magZ;

	errorCode = mlsPeriphImuGetAccel(&accelX, &accelY, &accelZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	uartData.floatValue = accelX;
	for(i = 0; i < 4; i++) {
		DataToSend[i+0] = uartData.byteArray[i];
	}
	uartData.floatValue = accelY;
	for(i = 0; i < 4; i++) {
		DataToSend[i+4] = uartData.byteArray[i];
	}
	uartData.floatValue = accelZ;
	for(i = 0; i < 4; i++) {
		DataToSend[i+8] = uartData.byteArray[i];
	}

	mlsPeriphImuGetGyro(&gyroX, &gyroY, &gyroZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	uartData.floatValue = gyroX;
	for(i = 0; i < 4; i++) {
		DataToSend[i+12] = uartData.byteArray[i];
	}
	uartData.floatValue = gyroY;
	for(i = 0; i < 4; i++) {
		DataToSend[i+16] = uartData.byteArray[i];
	}
	uartData.floatValue = gyroZ;
	for(i = 0; i < 4; i++) {
		DataToSend[i+20] = uartData.byteArray[i];
	}

	mlsPeriphImuGetMag(&magX, &magY, &magZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	uartData.floatValue = magX;
	for(i = 0; i < 4; i++) {
		DataToSend[i+24] = uartData.byteArray[i];
	}
	uartData.floatValue = magY;
	for(i = 0; i < 4; i++) {
		DataToSend[i+28] = uartData.byteArray[i];
	}
	uartData.floatValue = magZ;
	for(i = 0; i < 4; i++) {
		DataToSend[i+32] = uartData.byteArray[i];
	}

	errorCode = mlsPeriphUartSend(DataToSend);

	return errorCode;
}
#endif
/********** Class function implementation section *****************************/

/**@}*/
