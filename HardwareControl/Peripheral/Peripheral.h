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
/********** Constant  and compile switch definition section *******************/

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
mlsErrorCode_t periphImuInit(void);

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
mlsErrorCode_t periphImuGetAccel(float *accelX, float *accelY, float *accelZ);

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
mlsErrorCode_t periphImuGetGyro(float *gyroX, float *gyroY, float *gyroZ);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_PERIPHERAL_H_ */
/**@}*/
