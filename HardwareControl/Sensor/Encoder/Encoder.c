/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Encoder.c
 * @brief Encoder firmware
 *
 * Long description.
 * @date 2024-10-10
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "Encoder.h"
#include "HardwareInfo.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/
typedef struct encoder {
	uint32_t				maxReload;			/*!< Max reload value */
	uint8_t					isRun;				/*!< Running status */
	encoderFuncStart		startEnc;			/*!< Function start encoder */
	encoderFuncStop			stopEnc;			/*!< Function stop encoder */
	encoderFuncSetCounter	setCounter;			/*!< Function set counter */
	encoderFuncGetCounter	getCounter;			/*!< Function get counter */
	encoderFuncSetMode		setMode;			/*!< Function set mode counter */
} encoder_t;
/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
encoderHandle_t mlsEncoderInit(void)
{
	encoderHandle_t handle = calloc(1, sizeof(encoder_t));
	if (handle == NULL)
	{
		return NULL;
	}
	return handle;
}

mlsErrorCode_t mlsEncoderSetConfig(encoderHandle_t handle, encoderConfig_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->maxReload = config.maxReload;
	handle->isRun = 0;
	handle->getCounter = config.getCounter;
	handle->setCounter = config.setCounter;
	handle->startEnc = config.startEnc;
	handle->stopEnc = config.stopEnc;
	handle->setMode = config.setMode;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsEncoderStart(encoderHandle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_SUCCESS;
	errorCode = handle->startEnc();
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return errorCode;
}

mlsErrorCode_t mlsEncoderStop(encoderHandle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_SUCCESS;
	errorCode = handle->stopEnc();
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return errorCode;
}

mlsErrorCode_t mlsEncoderSetCounter(encoderHandle_t handle, uint32_t value)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_SUCCESS;
	errorCode = handle->setCounter(value);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return errorCode;
}

mlsErrorCode_t mlsEncoderGetCounter(encoderHandle_t handle, uint32_t *value)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_SUCCESS;
	errorCode = handle->getCounter(value);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return errorCode;
}

mlsErrorCode_t mlsEncoderSetMode(encoderHandle_t handle, uint8_t mode)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_SUCCESS;
	errorCode = handle->setMode(mode);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return errorCode;
}
/**@}*/
