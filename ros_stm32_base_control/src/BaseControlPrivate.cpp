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

#include "BaseControlPrivate.h"
#include "HardwareInfo.h"
/********** Local Constant and compile switch definition section **************/
#define USE_ROS_LOG_DEBUG

#define ROS_TOPIC_IMU                       "imu"
/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Local (static) variable definition section ************************/
ros::NodeHandle rosNodeHandle;    	/*!< ROS node handle */
//ros::Time RosCurrentTime;        	/*!< ROS current time */
char rosLogBuffer[100];          	/*!< ROS log message buffer */

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
void mlsBaseControlROSSetup(void)
{
    rosNodeHandle.initNode(); /*!< Init ROS node handle */
//    RosNodeHandle.advertise(chatter);
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

			logFlag = true;
		}
	}
	else
	{
		logFlag = false;
	}
}
/********** Class function implementation section *****************************/

/**@}*/

