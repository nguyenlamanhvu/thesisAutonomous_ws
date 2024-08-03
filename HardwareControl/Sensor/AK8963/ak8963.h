/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file ak8963.h
 * @brief AK8963 firmware
 *
 * Long description.
 * @date 2024-07-28
 * @author	Anh Vu
 */

#ifndef SENSOR_AK8963_AK8963_H_
#define SENSOR_AK8963_AK8963_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
#include "stdlib.h"
#include "string.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/
typedef mlsErrorCode_t (*ak8963FuncI2cWrite)(uint8_t regAddr, uint8_t *buffer, uint16_t len);
typedef mlsErrorCode_t (*ak8963FuncI2cRead)(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/**
 * @brief   Handle selection.
 */
typedef struct ak8963 *ak8963Handle_t;

/**
 * @brief   Mode selection.
 */
typedef enum {
	AK8963_MODE_PWR_DOWN = 0x00,                /*!< AK8963 mode power down */
	AK8963_MODE_SINGLE_MEASUREMENT = 0x01,      /*!< AK8963 mode single measurement */
	AK8963_MODE_CONT_MEASUREMENT_1 = 0x02,      /*!< AK8963 mode continous measurement 1 */
	AK8963_MODE_EXT_TRIG_MEASUREMENT = 0x04,    /*!< AK8963 mode external trigger measurement */
	AK8963_MODE_CONT_MEASUREMENT_2 = 0x06,      /*!< AK8963 mode continous measurement 2 */
	AK8963_MODE_SELF_TEST = 0x08,               /*!< AK8963 mode self test */
	AK8963_MODE_FUSE_ROM_ACCESS = 0x0F,         /*!< AK8963 mode fuse ROM access */
	AK8963_MODE_MAX
} ak8963Mode_t;

/**
 * @brief   Magnetometer full scale.
 */
typedef enum {
	AK8963_MFS_14BIT = 0,                       /*!< Magnetometer 14 bit resolution  */
	AK8963_MFS_16BIT,                           /*!< Magnetometer 16 bit resolution  */
	AK8963_MFS_MAX
} ak8963MfsSel_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	ak8963Mode_t  				oprerationMode; 			/*!< Operation mode */
	ak8963MfsSel_t  			mfsSel; 			  		/*!< Magnetometer full scale */
	float                       magHardIronBiasX;       	/*!< Magnetometer hard iron bias of x axis */
	float                       magHardIronBiasY;       	/*!< Magnetometer hard iron bias of y axis */
	float                       magHardIronBiasZ;       	/*!< Magnetometer hard iron bias of z axis */
	float                       magSoftIronBiasX;       	/*!< Magnetometer soft iron bias of x axis */
	float                       magSoftIronBiasY;       	/*!< Magnetometer soft iron bias of y axis */
	float                       magSoftIronBiasZ;       	/*!< Magnetometer soft iron bias of z axis */
	ak8963FuncI2cWrite        	i2cWrite;         			/*!< AK8963 write bytes */
	ak8963FuncI2cRead        	i2cRead;          			/*!< AK8963 read bytes */
} ak8963Config_t;

/********** Macro definition section*******************************************/
#define AK8963_ADDRESS   0x0C<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL1     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2	 0x0B
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
/********** Function declaration section **************************************/
/*
 * @brief   Initialize AK8963 with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
ak8963Handle_t mlsAk8963Init(void);

/*
 * @brief   Set configuration parameters.
 * @param 	handle: Handle structure.
 * @param   config: Configuration structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsAk8963SetConfig(ak8963Handle_t handle, ak8963Config_t config);

/*
 * @brief   Configure AK8963 to run.
 * @param 	handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsAk8963Config(ak8963Handle_t handle);

/*
 * @brief   Get magnetometer raw value.
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
mlsErrorCode_t mlsAk8963GetMagRaw(ak8963Handle_t handle, int16_t *rawX, int16_t *rawY, int16_t *rawZ);

/*
 * @brief   Get magnetometer calibrated data.
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
mlsErrorCode_t mlsAk8963GetMagCalib(ak8963Handle_t handle, float *calibX, float *calibY, float *calibZ);

/*
 * @brief   Get magnetometer scaled data.
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
mlsErrorCode_t mlsAk8963GetMagScale(ak8963Handle_t handle, float *scaleX, float *scaleY, float *scaleZ);

/*
 * @brief   Set hard iron bias data.
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
mlsErrorCode_t mlsAk8963SetHardIronBias(ak8963Handle_t handle, float biasX, float biasY, float biasZ);

/*
 * @brief   Get hard iron bias data.
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
mlsErrorCode_t mlsAk8963SGetHardIronBias(ak8963Handle_t handle, float *biasX, float *biasY, float *biasZ);

/*
 * @brief   Set soft iron bias data.
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
mlsErrorCode_t mlsAk8963SetSoftIronBias(ak8963Handle_t handle, float biasX, float biasY, float biasZ);

/*
 * @brief   Get soft iron bias data.
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
mlsErrorCode_t mlsAk8963GetSoftIronBias(ak8963Handle_t handle, float *biasX, float *biasY, float *biasZ);

/*
 * @brief   Auto calibrate magnetometer bias value.
 * @param   handle: Handle structure.
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsAk8963Calib3Axis(ak8963Handle_t handle);

/*
 * @brief   Destroy AK8963 pointer.
 * @param   handle: Handle structure.
 * @return	none
 */
void mlsAk8963Destroy(ak8963Handle_t handle);
#ifdef __cplusplus
}
#endif

#endif /* SENSOR_AK8963_AK8963_H_ */
/**@}*/
