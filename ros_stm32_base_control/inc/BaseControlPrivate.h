/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file BaseControlPrivate.h
 * @brief Private firmware communicates with ROS over rosserial protocol.
 *
 * Long description.
 * @date 2024-07-11
 * @author	Anh Vu
 */

#ifndef ROS_STM32_BASE_CONTROL_INC_BASECONTROLPRIVATE_H_
#define ROS_STM32_BASE_CONTROL_INC_BASECONTROLPRIVATE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "stdbool.h"
#include "stdint.h"

#include "errorCode.h"
#include "Peripheral.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/
#define LINEAR	0
#define ANGULAR	1
/********** Function declaration section **************************************/
/*
 * @brief Base Control ROS Setup
 * @param None
 * @return Error Code
 */
void mlsBaseControlROSSetup(void);

/*
 * @brief Base Control Spin Once
 * @param None
 * @return Error Code
 */
void mlsBaseControlSpinOnce(void);

/*
 * @brief Wait serial link
 * @param isConnected check Rosserial connect
 * @return Error Code
 */
void mlsBaseControlWaitSerialLink(bool isConnected);

/*
 * @brief Get connection status between base control and ROS node.
 * @param None
 * @return Bool
 * 			- True: Connected
 * 			- False: Disconnected
 */
bool mlsBaseControlConnectStatus(void);

/*
 * @brief Send log message to ROS node.
 * @param None
 * @return None
 */
void mlsBaseControlSendLogMsg(void);

/*
 * @brief Publish IMU message to ROS node.
 * @param None
 * @return None
 */
void mlsBaseControlPublishImuMsg(void);

/*
 * @brief Publish motor velocity message to ROS node.
 * @param None
 * @return None
 */
void mlsBaseControlPublishMortorVelocityMsg(void);

/*
 * @brief Update goal velocity.
 * @param None
 * @return None
 */
void mlsBaseControlUpdateGoalVel(void);

/*
 * @brief Publish IMU message to ROS node.
 * @param timeBasehandle
 * @return ErrorCode
 */
mlsErrorCode_t mlsBaseControlStartTimerInterrupt(TIM_HandleTypeDef* timBaseHandle);

#ifdef __cplusplus
}
#endif

#endif /* ROS_STM32_BASE_CONTROL_INC_BASECONTROLPRIVATE_H_ */
/**@}*/
