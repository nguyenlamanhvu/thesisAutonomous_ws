/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Matlab.h
 * @brief Driver for communicating with Matlab
 *
 * Long description.
 * @date 2024-09-25
 * @author	Anh Vu
 */

#ifndef UART_MATLAB_MATLAB_H_
#define UART_MATLAB_MATLAB_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
#include "stdlib.h"
#include "string.h"
#include "compilerSwitch.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/
typedef mlsErrorCode_t (*matlabFuncUartWrite)(uint8_t *buffer, uint16_t len);
typedef mlsErrorCode_t (*matlabFuncUartRead)(uint8_t *buffer, uint16_t len);

typedef struct matlab {
	uint8_t*					bufferSend;						/*!< Matlab buffer transmit */
	uint16_t					bufferSendLength;				/*!< Matlab buffer transmit length */
	uint8_t*					bufferRead;						/*!< Matlab buffer receive */
	uint16_t					bufferReadLength;				/*!< Matlab buffer receive length */
	matlabFuncUartWrite        	matlabWrite;         			/*!< Matlab write bytes */
	matlabFuncUartRead        	matlabRead;          			/*!< Matlab read bytes */
} matlab_t;

/**
 * @brief   Handle selection.
 */
typedef struct matlab *matlabHandle_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	uint8_t*					bufferSend;						/*!< Matlab buffer transmit */
	uint16_t					bufferSendLength;				/*!< Matlab buffer transmit length */
	uint8_t*					bufferRead;						/*!< Matlab buffer receive */
	uint16_t					bufferReadLength;				/*!< Matlab buffer receive length */
	matlabFuncUartWrite        	matlabWrite;         			/*!< Matlab write bytes */
	matlabFuncUartRead        	matlabRead;          			/*!< Matlab read bytes */
} matlabConfig_t;
/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Initialize Matlab with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
matlabHandle_t mlsMatlabInit(void);

/*
 * @brief   Set configuration parameters.
 * @param 	handle: Handle structure.
 * @param   config: Configuration structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMatlabSetConfig(matlabHandle_t handle, matlabConfig_t config);

/*
 * @brief   Send data to Matlab by using UART
 * @param 	handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMatlabSendData(matlabHandle_t handle);

/*
 * @brief   Read data to Matlab by using UART
 * @param 	handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMatlabReadData(matlabHandle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* UART_MATLAB_MATLAB_H_ */
/**@}*/
