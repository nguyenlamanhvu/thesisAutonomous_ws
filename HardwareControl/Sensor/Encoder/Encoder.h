/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Encoder.h
 * @brief Encoder firmware
 *
 * Long description.
 * @date 2024-10-10
 * @author	Anh Vu
 */


#ifndef SENSOR_ENCODER_ENCODER_H_
#define SENSOR_ENCODER_ENCODER_H_

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
typedef mlsErrorCode_t (*encoderFuncStart)(void);
typedef mlsErrorCode_t (*encoderFuncStop)(void);
typedef mlsErrorCode_t (*encoderFuncSetCounter)(uint32_t value);
typedef mlsErrorCode_t (*encoderFuncGetCounter)(uint32_t *value);
typedef mlsErrorCode_t (*encoderFuncSetMode)(uint8_t mode);

typedef struct encoder *encoderHandle_t;

typedef struct {
	uint32_t				maxReload;			/*!< Max reload value */
	encoderFuncStart		startEnc;			/*!< Function start encoder */
	encoderFuncStop			stopEnc;			/*!< Function stop encoder */
	encoderFuncSetCounter	setCounter;			/*!< Function set counter */
	encoderFuncGetCounter	getCounter;			/*!< Function get counter */
	encoderFuncSetMode		setMode;			/*!< Function set mode counter */
} encoderConfig_t;
/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Initialize Encoder with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
encoderHandle_t mlsEncoderInit(void);

/*
 * @brief   Set configuration parameters.
 * @param 	handle: Handle structure.
 * @param   config: Configuration structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsEncoderSetConfig(encoderHandle_t handle, encoderConfig_t config);

/*
 * @brief   Start encoder.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsEncoderStart(encoderHandle_t handle);

/*
 * @brief   Stop encoder.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsEncoderStop(encoderHandle_t handle);

/*
 * @brief   Set counter of Encoder.
 *
 * @param 	handle Handle structure.
 * @param 	value Counter value.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsEncoderSetCounter(encoderHandle_t handle, uint32_t value);

/*
 * @brief   Get counter of Encoder.
 *
 * @param 	handle Handle structure.
 * @param 	*value Counter value pointer.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsEncoderGetCounter(encoderHandle_t handle, uint32_t *value);

/*
 * @brief   Set mode of Encoder.
 *
 * @param 	handle Handle structure.
 * @param 	mode Encoder mode.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsEncoderSetMode(encoderHandle_t handle, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_ENCODER_ENCODER_H_ */
/**@}*/
