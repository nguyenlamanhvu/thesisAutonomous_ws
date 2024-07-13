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

#ifndef __BASECONTROLPRIVATE_H__
#define __BASECONTROLPRIVATE_H__

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "stdbool.h"
#include "stdint.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

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

#ifdef __cplusplus
}
#endif

#endif /* __BASECONTROLPRIVATE_H__ */
/**@}*/
