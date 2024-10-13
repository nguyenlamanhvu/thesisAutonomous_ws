/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Gui.c
 * @brief Firmware for using GUI
 *
 * Long description.
 * @date 2024-10-12
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "Gui.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global variable definition ********************************/
extern uint8_t gBufferRxData[UART_MAX_LENGTH];
extern dataFrame_t gGuiRxDataFrame;
extern dataFrame_t gGuiTxDataFrame;
extern UART_HandleTypeDef huart2;
/********** Global function definition section ********************************/
guiHandle_t mlsGuiInit(void)
{
	guiHandle_t handle = calloc(1, sizeof(gui_t));
	if (handle == NULL)
	{
		return NULL;
	}
	return handle;
}

mlsErrorCode_t mlsGuiSetConfig(guiHandle_t handle, guiConfig_t config)
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
	handle->guiRead = config.guiRead;
	handle->guiWrite = config.guiWrite;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsGuiConfig(guiHandle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	// start uart rx dma idle
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gBufferRxData, UART_MAX_LENGTH);

	return MLS_SUCCESS;
}


mlsErrorCode_t mlsGuiSendData(guiHandle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	/* Send data DMA*/
	return handle->guiWrite((uint8_t*)handle->bufferSend, handle->bufferSendLength);
}

mlsErrorCode_t mlsGuiReadData(guiHandle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	/* Read data DMA*/
	return handle->guiRead((uint8_t*)handle->bufferRead, handle->bufferReadLength);
}
/**@}*/
