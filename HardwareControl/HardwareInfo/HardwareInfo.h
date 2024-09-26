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
#include "BaseControl.h"
#include "main.h"
#include "compilerSwitch.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief Base Control Init
 * @param timeMs
 * @return Error Code
 */
void mlsHardwareInfoDelay(uint32_t timeMs);

/*
 * @brief Start timer interrupt
 * @param timBaseHandle
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoStartTimerInterrupt(TIM_HandleTypeDef* timBaseHandle);

/*
 * @brief Read data from MPU9250
 * @param[1] address of register in MPU9250
 * @param[2] buffer which store data
 * @param[3] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoMpu9250ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/*
 * @brief Write data to MPU9250
 * @param[1] address of register in MPU9250
 * @param[2] buffer which store data
 * @param[3] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoMpu9250WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/*
 * @brief Read data from AK8963
 * @param[1] address of register in AK8963
 * @param[2] buffer which store data
 * @param[3] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoAk8963ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/*
 * @brief Write data from AK8963
 * @param[1] address of register in AK8963
 * @param[2] buffer which store data
 * @param[3] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoAk8963WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/*
 * @brief Read data from UART
 * @param[1] buffer which store data
 * @param[2] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoUartReadBytes(uint8_t *buffer, uint16_t len);

/*
 * @brief Write data from UART
 * @param[1] buffer which store data
 * @param[2] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoUartWriteBytes(uint8_t *buffer, uint16_t len);
#ifdef __cplusplus
}
#endif

#endif /* HARDWARECONTROL_INC_HARDWAREINFO_H_ */
/**@}*/
