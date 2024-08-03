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
#include "ak8963.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/
#ifdef USE_ACC_GYRO_MPU9250

#define DEFAULT_ACCEL_BIAS_X				0
#define DEFAULT_ACCEL_BIAS_Y				0
#define DEFAULT_ACCEL_BIAS_Z				0

#define DEFAULT_GYRO_BIAS_X					0
#define DEFAULT_GYRO_BIAS_Y					0
#define DEFAULT_GYRO_BIAS_Z					0

#ifdef USE_MAGNETOMETER_MPU9250
#define DEFAULT_MAG_HARD_IRON_X				0
#define DEFAULT_MAG_HARD_IRON_Y				0
#define DEFAULT_MAG_HARD_IRON_Z				0

#define DEFAULT_MAG_SOFT_IRON_X				0
#define DEFAULT_MAG_SOFT_IRON_Y				0
#define DEFAULT_MAG_SOFT_IRON_Z				0
#endif

#endif
/********** Local (static) variable definition ********************************/
#ifdef USE_ACC_GYRO_MPU9250
mpu9250Handle_t mpu9250Handle = NULL;

#ifdef USE_MAGNETOMETER_MPU9250
ak8963Handle_t ak8963Handle = NULL;
#endif

#endif
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/

mlsErrorCode_t periphImuInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#ifdef USE_ACC_GYRO_MPU9250
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
			.xAxis = DEFAULT_ACCEL_BIAS_X,
			.yAxis = DEFAULT_ACCEL_BIAS_Y,
			.zAxis = DEFAULT_ACCEL_BIAS_Z
		},
		.gyroBias = {
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
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

#ifdef USE_MAGNETOMETER_MPU9250
	/* Initialize AK8963 pointer*/
	ak8963Handle = mlsAk8963Init();
	if(ak8963Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	ak8963Config_t ak8963Config = {
		.oprerationMode = AK8963_MODE_CONT_MEASUREMENT_1,
		.mfsSel = AK8963_MFS_16BIT,
		.magHardIronBiasX = DEFAULT_MAG_HARD_IRON_X,
		.magHardIronBiasY = DEFAULT_MAG_HARD_IRON_Y,
		.magHardIronBiasZ = DEFAULT_MAG_HARD_IRON_Z,
		.magSoftIronBiasX = DEFAULT_MAG_SOFT_IRON_X,
		.magSoftIronBiasY = DEFAULT_MAG_HARD_IRON_Y,
		.magSoftIronBiasZ = DEFAULT_MAG_SOFT_IRON_Z,
		.i2cWrite = mlsHardwareInfoAk8963WriteBytes,
		.i2cRead = mlsHardwareInfoAk8963ReadBytes
	};

	/* Set configuration for AK8963*/
	errorCode = mlsAk8963SetConfig(ak8963Handle, ak8963Config);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Configure Ak8963*/
	errorCode = mlsAk8963Config(ak8963Handle);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Calibrate AK8963 before get value*/
	errorCode = mlsAk8963Calib3Axis(ak8963Handle);

	return errorCode;
#endif

#elif USE_IMU_ADIS16488

#endif
}

mlsErrorCode_t periphImuGetAccel(float *accelX, float *accelY, float *accelZ)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#ifdef USE_ACC_GYRO_MPU9250
	errorCode = mlsMpu9250GetAccelScale(mpu9250Handle, accelX, accelY, accelZ);
#elif USE_IMU_ADIS16488

#endif
	return errorCode;
}

mlsErrorCode_t periphImuGetGyro(float *gyroX, float *gyroY, float *gyroZ)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#ifdef USE_ACC_GYRO_MPU9250
	errorCode = mlsMpu9250GetGyroScale(mpu9250Handle, gyroX, gyroY, gyroZ);
#elif USE_IMU_ADIS16488

#endif
	return errorCode;
}

mlsErrorCode_t periphImuGetMag(float *magX, float *magY, float *magZ)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#if defined(USE_ACC_GYRO_MPU9250) && defined(USE_MAGNETOMETER_MPU9250)
	errorCode = mlsAk8963GetMagScale(ak8963Handle, magX, magY, magZ);
#elif USE_IMU_ADIS16488

#endif
	return errorCode;
}

/**@}*/
