/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Gui.h
 * @brief Firmware for using GUI
 *
 * Long description.
 * @date 2024-10-12
 * @author	Anh Vu
 */


#ifndef UART_GUI_GUI_H_
#define UART_GUI_GUI_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
#include "stdlib.h"
#include "string.h"
#include "compilerSwitch.h"
#include "HardwareInfo.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/
typedef mlsErrorCode_t (*guiFuncUartWrite)(uint8_t *buffer, uint16_t len);
typedef mlsErrorCode_t (*guiFuncUartRead)(uint8_t *buffer, uint16_t len);

typedef struct gui {
	dataFrame_t*				bufferSend;						/*!< GUI buffer transmit */
	uint16_t					bufferSendLength;				/*!< GUI buffer transmit length */
	dataFrame_t*				bufferRead;						/*!< GUI buffer receive */
	uint16_t					bufferReadLength;				/*!< GUI buffer receive length */
	guiFuncUartWrite        	guiWrite;         				/*!< GUI write bytes */
	guiFuncUartRead	        	guiRead;          				/*!< GUI read bytes */
} gui_t;

/**
 * @brief   Handle selection.
 */
typedef struct gui *guiHandle_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	dataFrame_t*				bufferSend;						/*!< GUI buffer transmit */
	uint16_t					bufferSendLength;				/*!< GUI buffer transmit length */
	dataFrame_t*				bufferRead;						/*!< GUI buffer receive */
	uint16_t					bufferReadLength;				/*!< GUI buffer receive length */
	guiFuncUartWrite        	guiWrite;         				/*!< GUI write bytes */
	guiFuncUartRead	        	guiRead;          				/*!< GUI read bytes */
} guiConfig_t;
/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Initialize GUI with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
guiHandle_t mlsGuiInit(void);

/*
 * @brief   Set configuration parameters.
 * @param 	handle: Handle structure.
 * @param   config: Configuration structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsGuiSetConfig(guiHandle_t handle, guiConfig_t config);

/*
 * @brief   Configure with default parameters.
 * @param 	handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsGuiConfig(guiHandle_t handle);

/*
 * @brief   Send data to GUI by using UART
 * @param 	handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsGuiSendData(guiHandle_t handle);

/*
 * @brief   Read data to GUI by using UART
 * @param 	handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsGuiReadData(guiHandle_t handle);
#ifdef __cplusplus
}
#endif

#endif /* UART_GUI_GUI_H_ */
/**@}*/
