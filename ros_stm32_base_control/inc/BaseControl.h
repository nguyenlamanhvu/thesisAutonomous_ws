/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file BaseControl.h
 * @brief Firmware communicates with ROS over rosserial protocol.
 *
 * Long description.
 * @date 2024-07-11
 * @author	Anh Vu
 */

#ifndef __BASECONTROL_PUBLIC_H__
#define __BASECONTROL_PUBLIC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/

/*
 * @brief Base Control Init
 * @return Error Code
 */
mlsErrorCode_t mlsBaseControlInit(void);

/*
 * @brief Base Control Main
 * @return Error Code
 */
mlsErrorCode_t mlsBaseControlMain(void);

#ifdef __cplusplus
}
#endif

#endif /* __BASECONTROL_PUBLIC_H__ */
/**@}*/
