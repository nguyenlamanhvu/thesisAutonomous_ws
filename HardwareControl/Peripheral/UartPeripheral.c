/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file UartPeripheral.c
 * @brief Library about uart peripheral
 *
 * Long description.
 * @date 2024-09-25
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "Peripheral.h"
#include "HardwareInfo.h"
#include "Matlab.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/
#define MATLAB_MAX_LENGTH 	36
/********** Local (static) variable definition ********************************/
#if (USE_UART_MATLAB == 1)
matlabHandle_t matlabHandle = NULL;
#endif
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global variable definition ********************************/
uint8_t *gBufferSend;
uint8_t *gBufferRead;
/********** Global function definition section ********************************/
mlsErrorCode_t mlsPeriphUartInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#if (USE_UART_MATLAB == 1)
	/* Initialize MATLAB pointer*/
	matlabHandle = mlsMatlabInit();
	if(matlabHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	matlabConfig_t matlabConfig = {
			.bufferRead = gBufferRead,
			.bufferSend = gBufferSend,
			.bufferReadLength = MATLAB_MAX_LENGTH,
			.bufferSendLength = MATLAB_MAX_LENGTH,
			.matlabRead = mlsHardwareInfoUartReadBytes,
			.matlabWrite = mlsHardwareInfoUartWriteBytes
	};

	/* Set configuration for UART MATLAB*/
	errorCode = mlsMatlabSetConfig(matlabHandle, matlabConfig);
#endif
	return errorCode;
}

mlsErrorCode_t mlsPeriphUartSend(uint8_t *data)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#if (USE_UART_MATLAB == 1)
	matlabHandle->bufferSendLength = sizeof(data);
	memcpy(matlabHandle->bufferSend,data,matlabHandle->bufferSendLength);
	errorCode = mlsMatlabSendData(matlabHandle);
#endif
	return errorCode;
}

mlsErrorCode_t mlsPeriphUartRead(uint8_t *data)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#if (USE_UART_MATLAB == 1)
	matlabHandle->bufferReadLength = sizeof(data);
	memcpy(matlabHandle->bufferRead,data,matlabHandle->bufferReadLength);
	errorCode = mlsMatlabReadData(matlabHandle);
#endif
	return errorCode;
}
/**@}*/
