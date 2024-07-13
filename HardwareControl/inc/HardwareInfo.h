/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file HardwareInfo.h
 * @brief Information of STM32F407
 *
 * Long description.
 * @date 2024-07-11
 * @author	Anh Vu
 */


#ifndef HARDWARECONTROL_INC_HARDWAREINFO_H_
#define HARDWARECONTROL_INC_HARDWAREINFO_H_

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
void mlsHardwareInfoDelay(uint32_t time_ms);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARECONTROL_INC_HARDWAREINFO_H_ */
/**@}*/
