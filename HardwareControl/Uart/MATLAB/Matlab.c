/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Matlab.c
 * @brief Driver for communicating with Matlab
 *
 * Long description.
 * @date 2024-09-25
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "Matlab.h"
#include "HardwareInfo.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
matlabHandle_t mlsMatlabInit(void)
{
	matlabHandle_t handle = calloc(1, sizeof(matlab_t));
	if (handle == NULL)
	{
		return NULL;
	}
	return handle;
}

mlsErrorCode_t mlsMatlabSetConfig(matlabHandle_t handle, matlabConfig_t config)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->bufferRead = config.bufferRead;
	handle->bufferReadLength = config.bufferReadLength;
	handle->bufferSend = config.bufferSend;
	handle->bufferSendLength = config.bufferSendLength;
	handle->matlabRead = config.matlabRead;
	handle->matlabWrite = config.matlabWrite;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMatlabSendData(matlabHandle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	/* Send data DMA*/
	return handle->matlabWrite(handle->bufferSend, handle->bufferSendLength);
}

mlsErrorCode_t mlsMatlabReadData(matlabHandle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	/* Send data DMA*/
	return handle->matlabRead(handle->bufferRead, handle->bufferReadLength);
}
/**@}*/
