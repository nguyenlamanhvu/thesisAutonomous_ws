/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file mpu9250.h
 * @brief MPU9250 firmware
 *
 * Long description.
 * @date 2024-07-22
 * @author	Anh Vu
 */


#ifndef SENSOR_MPU9250_MPU9250_H_
#define SENSOR_MPU9250_MPU9250_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
#include "compilerSwitch.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/
typedef mlsErrorCode_t (*mpu9250FuncI2cRead)(uint8_t regAddr, uint8_t *buffer, uint16_t len);
typedef mlsErrorCode_t (*mpu9250FuncI2cWrite)(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/**
 * @brief   Handle structure.
 */
typedef struct mpu9250 *mpu9250Handle_t;

/**
 * @brief   Accelerometer bias.
 */
typedef struct {
    int16_t xAxis;
    int16_t yAxis;
    int16_t zAxis;
} mpu9250AccelBias_t;

/**
 * @brief   Gyroscope bias.
 */
typedef struct {
    int16_t xAxis;
    int16_t yAxis;
    int16_t zAxis;
} mpu9250GyroBias_t;

/**
 * @brief   Clock selection.
 */
typedef enum {
    MPU9250_CLKSEL_INTERNAL_20_MHZ = 0,     /*!< Internal 20 MHz clock source */
    MPU9250_CLKSEL_AUTO,                    /*!< Auto select best available clock source */
    MPU9250_CLKSEL_MAX
} mpu9250Clksel_t;

/**
 * @brief   Low pass filter.
 */
typedef enum {
    MPU9250_250ACEL_4000GYRO_BW_HZ = 0,     /*!< 250 Hz accelerometer bandwidth, 4000 Hz gyroscope bandwidth */
    MPU9250_184ACEL_188GYRO_BW_HZ,          /*!< 184 Hz accelerometer bandwidth, 188 Hz gyroscope bandwidth */
    MPU9250_92ACEL_98GYRO_BW_HZ,            /*!< 92 Hz accelerometer bandwidth, 98 Hz gyroscope bandwidth */
    MPU9250_41ACEL_42GYRO_BW_HZ,            /*!< 41 Hz accelerometer bandwidth, 42 Hz gyroscope bandwidth */
    MPU9250_20ACEL_20GYRO_BW_HZ,            /*!< 20 Hz accelerometer bandwidth, 20 Hz gyroscope bandwidth */
    MPU9250_10ACEL_10GYRO_BW_HZ,            /*!< 10 Hz accelerometer bandwidth, 10 Hz gyroscope bandwidth */
    MPU9250_5ACEL_5GYRO_BW_HZ,              /*!< 5 Hz accelerometer bandwidth, 5 Hz gyroscope bandwidth */
    MPU9250_DLPF_CFG_MAX
} mpu9250DlpfConfig_t;

/**
 * @brief   Sleep mode.
 */
typedef enum {
    MPU9250_DISABLE_SLEEP_MODE = 0,         /*!< Disable sleep mode */
    MPU9250_LOW_PWR_SLEEP_MODE,             /*!< Low power mode */
    MPU9250_SLEEP_MODE_MAX
} mpu9250SleepMode_t;

/**
 * @brief   Gyroscope full scale.
 */
typedef enum {
    MPU9250_FS_SEL_250 = 0,                 /*!< 250 deg/s */
    MPU9250_FS_SEL_500,                     /*!< 500 deg/s */
    MPU9250_FS_SEL_1000,                    /*!< 1000 deg/s */
    MPU9250_FS_SEL_2000,                    /*!< 2000 deg/s */
    MPU9250_FS_SEL_MAX
} mpu9250FsSel_t;

/**
 * @brief   Accelerometer full scale.
 */
typedef enum {
    MPU9250_AFS_SEL_2G = 0,                 /*!< 2g */
    MPU9250_AFS_SEL_4G,                     /*!< 4g */
    MPU9250_AFS_SEL_8G,                     /*!< 8g */
    MPU9250_AFS_SEL_16G,                    /*!< 16g */
    MPU9250_AFS_SEL_MAX
} mpu9250AfsSel_t;

/**
 * @brief   Configuration structure.
 */
typedef enum {
    MPU9250_COMM_MODE_I2C = 0,              /*!< Interface over I2C */
    MPU9250_COMM_MODE_SPI,                  /*!< Interface over SPI */
    MPU9250_COMM_MODE_MAX
} mpu9250CommMode_t;

typedef struct {
    mpu9250Clksel_t        	clkSel;         /*!< MPU9250 clock source */
    mpu9250DlpfConfig_t     dlpfConfig;     /*!< MPU9250 digital low pass filter (DLPF) */
    mpu9250SleepMode_t    	sleepMode;     	/*!< MPU9250 sleep mode */
    mpu9250FsSel_t        	fsSel;         	/*!< MPU9250 gyroscope full scale range */
    mpu9250AfsSel_t       	afsSel;        	/*!< MPU9250 accelerometer full scale range */
    mpu9250AccelBias_t    	accelBias;     	/*!< Accelerometer bias */
    mpu9250GyroBias_t     	gyroBias;      	/*!< Gyroscope bias */
    mpu9250FuncI2cRead		i2cRead;		/*!< MPU9250 read bytes*/
    mpu9250FuncI2cWrite		i2cWrite;		/*!< MPU9250 write bytes*/
    mpu9250CommMode_t     	commMode;      	/*!< Interface protocol */
} mpu9250Config_t;

/********** Macro definition section*******************************************/

//MPU9250 register map
#define MPU9250_SELF_TEST_X_GYRO        0x00        /*!< Gyroscope self-test registers */
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02
#define MPU9250_SELF_TEST_X_ACCEL       0x0D        /*!< Accelerometer self-test registers */
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F
#define MPU9250_XG_OFFSET_H             0x13        /*!< Gyroscope offset registers */
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x14
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPRT_DIV               0x19        /*!< Sample rate divider */
#define MPU9250_CONFIG                  0x1A        /*!< Configuration */
#define MPU9250_GYRO_CONFIG             0x1B        /*!< Gyroscope configuration */
#define MPU9250_ACCEL_CONFIG            0x1C        /*!< Accelerometer configuration */
#define MPU9250_ACCEL_CONFIG2           0x1D        /*!< Accelerometer configuration 2 */
#define MPU9250_LP_ACCEL_ODR            0x1E        /*!< Low power accelerometer ODR control */
#define MPU9250_WOM_THR                 0x1F        /*!< Wake-on motion threshold */
#define MPU9250_FIFO_EN                 0x23        /*!< FIFO enable */
#define MPU9250_I2C_MST_CTRL            0x24        /*!< I2C master control */
#define MPU9250_I2C_SLV0_ADDR           0x25        /*!< I2C slave 0 control */
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28        /*!< I2C slave 1 control */
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B        /*!< I2C slave 2 control */
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E        /*!< I2C slave 3 control */
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31        /*!< I2C slave 4 control */
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36        /*!< I2C master status */
#define MPU9250_INT_PIN_CFG             0x37        /*!< Interrupt pin/bypass enable configuration */
#define MPU9250_INT_ENABLE              0x38        /*!< Interrupt enable */
#define MPU9250_INT_STATUS              0x3A        /*!< Interrupt status */
#define MPU9250_ACCEL_XOUT_H            0x3B        /*!< Accelerometer measurements */
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41        /*!< Temperature measurements */
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43        /*!< Gyroscope measurements */
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49        /*!< External sensor data */
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60
#define MPU9250_I2C_SLV0_DO             0x63        /*!< I2C slave 0 data out */
#define MPU9250_I2C_SLV1_DO             0x64        /*!< I2C slave 1 data out */
#define MPU9250_I2C_SLV2_DO             0x65        /*!< I2C slave 2 data out */
#define MPU9250_I2C_SLV3_DO             0x66        /*!< I2C slave 3 data out */
#define MPU9250_I2C_MST_DELAY_CTRL      0x67        /*!< I2C master delay control */
#define MPU9250_SIGNAL_PATH_RESET       0x68        /*!< Signal path reset */
#define MPU9250_MOT_DETECT_CTRL         0x69        /*!< Accelerometer interrupt control */
#define MPU9250_USER_CTRL               0x6A        /*!< User control */
#define MPU9250_PWR_MGMT_1              0x6B        /*!< Power management 1 */
#define MPU9250_PWR_MGMT_2              0x6C        /*!< Power management 2 */
#define MPU9250_FIFO_COUNTH             0x72        /*!< FIFO counter registers */
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFP_R_W                0x74        /*!< FIFO read write */
#define MPU9250_WHO_AM_I                0x75        /*!< Who am I */
#define MPU9250_XA_OFFSET_H             0x77        /*!< Accelerometer offset registers */
#define MPU9250_XA_OFFSET_L             0x78
#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B
#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E
#define MPU9250_ADDR                    (0x68<<1)   /*!< MPU9250 Address */

#define BUFFER_CALIB_DEFAULT			1000		/*!< 1000 Samples*/
/********** Function declaration section **************************************/
/*
 * @brief   Initialize MPU9050 with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mpu9250Handle_t mlsMpu9250Init(void);

/*
 * @brief   Set configuration parameters.
 * @param 	handle: Handle structure.
 * @param   config: Configuration structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250SetConfig(mpu9250Handle_t handle, mpu9250Config_t config);

/*
 * @brief   Configure MPU6050 to run.
 * @param 	handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250Config(mpu9250Handle_t handle);

/*
 * @brief   Get accelerometer raw value.
 *
 * @param   handle: Handle structure.
 * @param   rawX: Raw value x axis.
 * @param   rawY: Raw value y axis.
 * @param   rawZ: Raw value z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250GetAccelRaw(mpu9250Handle_t handle, int16_t *rawX, int16_t *rawY, int16_t *rawZ);

/*
 * @brief   Get accelerometer calibrated data.
 *
 * @param   handle: Handle structure.
 * @param   calibX: Calibrated data x axis.
 * @param   calibY: Calibrated data y axis.
 * @param   calibZ: Calibrated data z axis.
 *
 * @return
 *      - MLS_SUCCESS:		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250GetAccelCalib(mpu9250Handle_t handle, int16_t *calibX, int16_t *calibY, int16_t *calibZ);

/*
 * @brief   Get accelerometer scaled data.
 *
 * @param   handle: Handle structure.
 * @param   scaleX: Scaled data x axis.
 * @param   scaleY: Scaled data y axis.
 * @param   scaleZ: Scaled data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250GetAccelScale(mpu9250Handle_t handle, float *scaleX, float *scaleY, float *scaleZ);

/*
 * @brief   Get gyroscope raw value.
 *
 * @param   handle: Handle structure.
 * @param   rawX: Raw value x axis.
 * @param   rawY: Raw value y axis.
 * @param   rawZ: Raw value z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250GetGyroRaw(mpu9250Handle_t handle, int16_t *rawX, int16_t *rawY, int16_t *rawZ);

/*
 * @brief   Get gyroscope calibrated data.
 *
 * @param   handle: Handle structure.
 * @param   calibX: Calibrated data x axis.
 * @param   calibY: Calibrated data y axis.
 * @param   calibZ: Calibrated data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250GetGyroCalib(mpu9250Handle_t handle, int16_t *calibX, int16_t *calibY, int16_t *calibZ);

/*
 * @brief   Get gyroscope scaled data.
 *
 * @param   handle: Handle structure.
 * @param   scaleX: Scaled data x axis.
 * @param   scaleY: Scaled data y axis.
 * @param   scaleZ: Scaled data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250GetGyroScale(mpu9250Handle_t handle, float *scaleX, float *scaleY, float *scaleZ);

/*
 * @brief   Set accelerometer bias data.
 *
 * @param   handle: Handle structure.
 * @param   biasX: Bias data x axis.
 * @param   biasY: Bias data y axis.
 * @param   biasZ: Bias data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250SetAccelBias(mpu9250Handle_t handle, int16_t biasX, int16_t biasY, int16_t biasZ);

/*
 * @brief   Set gyroscope bias data.
 *
 * @param   handle: Handle structure.
 * @param   biasX: Bias data x axis.
 * @param   biasY: Bias data y axis.
 * @param   biasZ: Bias data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250SetGyroBias(mpu9250Handle_t handle, int16_t biasX, int16_t biasY, int16_t biasZ);

/*
 * @brief   Get accelerometer bias data.
 *
 * @param   handle: Handle structure.
 * @param   biasX: Bias data x axis.
 * @param   biasY: Bias data y axis.
 * @param   biasZ: Bias data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250GetAccelBias(mpu9250Handle_t handle, int16_t *biasX, int16_t *biasY, int16_t *biasZ);

/*
 * @brief   Get gyroscope bias data.
 *
 * @param   handle: Handle structure.
 * @param   biasX: Bias data x axis.
 * @param   biasY: Bias data y axis.
 * @param   biasZ: Bias data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250GetGyroBias(mpu9250Handle_t handle, int16_t *biasX, int16_t *biasY, int16_t *biasZ);

/*
 * @brief   Auto calibrate all accelerometer and gyroscope bias value.
 * @param   handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMpu9250Calib6Axis(mpu9250Handle_t handle);

/*
 * @brief   Destroy MPU9250 pointer.
 * @param   handle: Handle structure.
 * @return	none
 */
void mlsMpu9250Destroy(mpu9250Handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MPU9250_MPU9250_H_ */
/**@}*/
