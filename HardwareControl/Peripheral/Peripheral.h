/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Peripheral.h
 * @brief Library of peripheral
 *
 * Long description.
 * @date 2024-07-28
 * @author	Anh Vu
 */

#ifndef PERIPHERAL_PERIPHERAL_H_
#define PERIPHERAL_PERIPHERAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
#include "compilerSwitch.h"
/********** Constant  and compile switch definition section *******************/
#define USE_MAGNETOMETER_MPU9250
#define USE_ACC_GYRO_MPU9250
//#define USE_IMU_ADIS16488

/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Initialize IMU with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuInit(void);

/*
 * @brief   Get accelerometer data.
 *
 * @param   handle: Handle structure.
 * @param   accelX: Accelerometer data x axis.
 * @param   accelY: Accelerometer data y axis.
 * @param   accelZ: Accelerometer data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuGetAccel(float *accelX, float *accelY, float *accelZ);

/*
 * @brief   Get gyroscope data.
 *
 * @param   handle: Handle structure.
 * @param   gyroX: gyroscope data x axis.
 * @param   gyroY: gyroscope data y axis.
 * @param   gyroZ: gyroscope data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuGetGyro(float *gyroX, float *gyroY, float *gyroZ);

/*
 * @brief   Get magnetometer data.
 *
 * @param   handle: Handle structure.
 * @param   magX: magnetometer data x axis.
 * @param   magY: magnetometer data y axis.
 * @param   magZ: magnetometer data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuGetMag(float *magX, float *magY, float *magZ);

/*
 * @brief   Initialize UART with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphUartInit(void);

/*
 * @brief   Send data by using uart.
 *
 * @param   data: Data string.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphUartSend(uint8_t *data);

/*
 * @brief   Read data by using uart.
 *
 * @param   data: Data string.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphUartRead(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_PERIPHERAL_H_ */
/**@}*/
