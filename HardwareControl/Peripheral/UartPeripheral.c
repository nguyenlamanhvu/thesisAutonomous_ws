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
#include "Gui.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/
#if (USE_UART_MATLAB == 1)
matlabHandle_t matlabHandle = NULL;
#elif (USE_UART_GUI == 1)
guiHandle_t guiHandle = NULL;
#endif
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global variable definition ********************************/
#if (USE_UART_MATLAB == 1)

#elif (USE_UART_GUI == 1)
extern dataFrame_t gGuiRxDataFrame;
extern dataFrame_t gGuiTxDataFrame;
#endif
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
			.bufferRead = &gGuiRxDataFrame,
			.bufferSend = &gGuiTxDataFrame,
			.bufferReadLength = UART_MAX_LENGTH,
			.bufferSendLength = UART_MAX_LENGTH,
			.matlabRead = mlsHardwareInfoUartReadBytes,
			.matlabWrite = mlsHardwareInfoUartWriteBytes
	};

	/* Set configuration for UART MATLAB*/
	errorCode = mlsMatlabSetConfig(matlabHandle, matlabConfig);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Configure for UART MATLAB*/
	errorCode = mlsMatlabConfig(matlabHandle);
#elif (USE_UART_GUI == 1)
	/* Initialize GUI pointer*/
	guiHandle = mlsGuiInit();
	if(guiHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	guiConfig_t guiConfig = {
			.bufferRead = &gGuiRxDataFrame,
			.bufferSend = &gGuiTxDataFrame,
			.bufferReadLength = UART_MAX_LENGTH,
			.bufferSendLength = UART_MAX_LENGTH,
			.guiRead = mlsHardwareInfoUartReadBytes,
			.guiWrite = mlsHardwareInfoUartWriteBytes
	};

	/* Set configuration for UART GUI*/
	errorCode = mlsGuiSetConfig(guiHandle, guiConfig);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Configure for UART GUI*/
	errorCode = mlsGuiConfig(guiHandle);
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
#elif (USE_UART_GUI == 1)
	guiHandle->bufferSendLength = sizeof(data);
	memcpy((uint8_t*)guiHandle->bufferSend, data, guiHandle->bufferSendLength);
	errorCode = mlsGuiSendData(guiHandle);
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
#elif (USE_UART_GUI == 1)

#endif
	return errorCode;
}
/**@}*/
