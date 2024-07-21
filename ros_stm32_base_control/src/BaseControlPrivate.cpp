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

#define ROS_TOPIC_IMU                       "imu"
#define ROS_TOPIC_VEL						"robot_vel"
#define ROS_TOPIC_CALLBACK_VEL				"callback_robot_vel"
/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/

/********** Local (static) function declaration section ***********************/
static sensor_msgs::Imu BaseControlGetIMU(void);
static ros::Time BaseControlGetROSTime(void);
static void BaseControlCallbackCommandVelocity(const geometry_msgs::Twist &callbackVelMsg);

/********** Local (static) variable definition section ************************/
ros::NodeHandle rosNodeHandle;    	/*!< ROS node handle */
char rosLogBuffer[100];          	/*!< ROS log message buffer */

char imuFrameId[20];

sensor_msgs::Imu imuMsg;           	/*!< ROS IMU message */
geometry_msgs::Twist velocityMsg;	/*!< ROS velocity message*/
nav_msgs::Odometry odometryMsg;		/*!< ROS odometry message*/

ros::Subscriber<geometry_msgs::Twist> callbackVelSub(ROS_TOPIC_CALLBACK_VEL, BaseControlCallbackCommandVelocity);

ros::Publisher imuPub(ROS_TOPIC_IMU, &imuMsg);
ros::Publisher velPub(ROS_TOPIC_VEL, &velocityMsg);

float goalVelocity[2] = {0.0, 0.0};            	/*!< Velocity to control motor */
float goalReceiveVelocity[2] = {0.0, 0.0};   	/*!< Velocity receive from "callback_robot_vel" topic */
/********** Local function definition section *********************************/
static sensor_msgs::Imu BaseControlGetIMU(void)
{
	float accelX = 9.78, accelY = 0, accelZ = 0;
	float gyroX = 0, gyroY = 0, gyroZ = 0;
	float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
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
/********** Class function implementation section *****************************/

/**@}*/
