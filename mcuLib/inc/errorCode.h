/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file errorCode.h
 * @brief Library about error code
 *
 * Long description.
 * @date 2024-07-13
 * @author	Anh Vu
 */


#ifndef MCULIB_INC_ERRORCODE_H_
#define MCULIB_INC_ERRORCODE_H_

#ifdef __cplusplus
extern "C" {
#endif

/********** Include section ***************************************************/
#ifdef STM32F103xB
#include "stm32f1xx.h"
#endif

#if defined(STM32F407xx) || defined(STM32F405xx)
#include "stm32f4xx.h"
#endif

#ifdef STM32L4R5xx
#include "stm32l4xx.h"
#endif
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/
typedef uint32_t mlsErrorCode_t;
/********** Macro definition section*******************************************/
#define MLS_SUCCESS 		HAL_OK
#define MLS_ERROR 			HAL_ERROR
#define MLS_BUSY			HAL_BUSY
#define MLS_TIMEOUT			HAL_TIMEOUT
#define MLS_ERROR_NULL_PTR 	(uint32_t)0xFFFE
/********** Function declaration section **************************************/

#ifdef __cplusplus
}
#endif

#endif /* MCULIB_INC_ERRORCODE_H_ */
/**@}*/
