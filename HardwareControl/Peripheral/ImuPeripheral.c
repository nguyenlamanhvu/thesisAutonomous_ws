/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file ImuPeripheral.c
 * @brief Library about peripheral for IMU
 *
 * Long description.
 * @date 2024-07-28
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "Peripheral.h"
#include "HardwareInfo.h"
#include "mpu9250.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/
#define DEFAULT_ACCEL_BIAS_X				0
#define DEFAULT_ACCEL_BIAS_Y				0
#define DEFAULT_ACCEL_BIAS_Z				0

#define DEFAULT_GYRO_BIAS_X					0
#define DEFAULT_GYRO_BIAS_Y					0
#define DEFAULT_GYRO_BIAS_Z					0
/********** Local (static) variable definition ********************************/
mpu9250Handle_t mpu9250Handle = NULL;
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
mlsErrorCode_t periphImuInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	/* Initialize MPU9250 pointer*/
	mpu9250Handle = mlsMpu9250Init();
	if(mpu9250Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mpu9250Config_t mpu9250Config = {
		.clkSel = MPU9250_CLKSEL_AUTO,
		.dlpfConfig = MPU9250_250ACEL_4000GYRO_BW_HZ,
		.sleepMode = MPU9250_DISABLE_SLEEP_MODE,
		.fsSel = MPU9250_FS_SEL_2000,
		.afsSel = MPU9250_AFS_SEL_2G,
		.accelBias = {
			.xAxis = DEFAULT_GYRO_BIAS_X,
			.yAxis = DEFAULT_GYRO_BIAS_Y,
			.zAxis = DEFAULT_GYRO_BIAS_Z
		},
		.i2cWrite = mlsHardwareInfoMpu9250WriteBytes,
		.i2cRead = mlsHardwareInfoMpu9250ReadBytes
	};

	/* Set configuration for MPU9250*/
	errorCode = mlsMpu9250SetConfig(mpu9250Handle, mpu9250Config);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Configure MPU9250*/
	errorCode = mlsMpu9250Config(mpu9250Handle);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Calibrate MPU9250 before get value*/
	errorCode = mlsMpu9250Calib6Axis(mpu9250Handle);

	return errorCode;
}

mlsErrorCode_t periphImuGetAccel(float *accelX, float *accelY, float *accelZ)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMpu9250GetAccelScale(mpu9250Handle, accelX, accelY, accelZ);

	return errorCode;
}

mlsErrorCode_t periphImuGetGyro(float *gyroX, float *gyroY, float *gyroZ)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMpu9250GetGyroScale(mpu9250Handle, gyroX, gyroY, gyroZ);

	return errorCode;
}

/**@}*/
