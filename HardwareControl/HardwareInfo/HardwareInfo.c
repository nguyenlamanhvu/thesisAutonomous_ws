/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file <fileName>.c
 * @brief <Brief description>
 *
 * Long description.
 * @date 2024-07-13
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "HardwareInfo.h"
#include "mpu9250.h"
#include "ak8963.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/
#define TIMER_MAX_RELOAD 				0xFFFF
#define HW_LEFTMOTOR_TIM_HANDLE 		htim1
#define HW_LEFTMOTOR_TIM 				TIM1
#define HW_LEFTMOTOR_TIM_CCR 			CCR1
#define HW_LEFTMOTOR_TIM_CHANNEL 		TIM_CHANNEL_1
#define HW_LEFTMOTOR_TIM_CLK_FREQ 		168000000
#define HW_LEFTMOTOR_GPIO 				GPIOE
#define HW_LEFTMOTOR_GPIO_PIN 			GPIO_PIN_10

#define HW_RIGHTMOTOR_TIM_HANDLE 		htim1
#define HW_RIGHTMOTOR_TIM 				TIM1
#define HW_RIGHTMOTOR_TIM_CCR 			CCR1
#define HW_RIGHTMOTOR_TIM_CHANNEL 		TIM_CHANNEL_2
#define HW_RIGHTMOTOR_TIM_CLK_FREQ 		168000000
#define HW_RIGHTMOTOR_GPIO 				GPIOE
#define HW_RIGHTMOTOR_GPIO_PIN 			GPIO_PIN_12
/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/
extern uint8_t gBaseControlTimeUpdateFlag[10];
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
/********** Local (static) variable definition ********************************/
uint32_t gBaseControlTimeUpdate[10];
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6)
  {
    if(gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX] >= 1000/IMU_PUBLISH_FREQUENCY)
    {
    	gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] = 1;
    	gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX] = 0;
    }

    if(gBaseControlTimeUpdate[IMU_UPDATE_TIME_INDEX] >= 1000/IMU_UPDATE_FREQUENCY)
	{
    	gBaseControlTimeUpdateFlag[IMU_UPDATE_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[IMU_UPDATE_TIME_INDEX] = 0;
	}

    if(gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX] >= 1000/CONTROL_MOTOR_FREQUENCY)
    {
    	gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX] = 0;
    }

    if(gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX] >= 1000/VEL_PUBLISH_FREQUENCY)
	{
		gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX] = 0;
	}

    if(gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX] >= 1000/DRIVE_INFORMATION_FREQUENCY)
	{
		gBaseControlTimeUpdateFlag[DRIVE_INFORMATION_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX] = 0;
	}

    gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX]++;
	gBaseControlTimeUpdate[IMU_UPDATE_TIME_INDEX]++;
	gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX]++;
	gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX]++;
	gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX]++;
  }
}

/********** Global function definition section ********************************/
void mlsHardwareInfoDelay(uint32_t timeMs)
{
	HAL_Delay(timeMs);
}

mlsErrorCode_t mlsHardwareInfoStartTimerInterrupt(TIM_HandleTypeDef* timBaseHandle)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = (mlsErrorCode_t)HAL_TIM_Base_Start_IT(timBaseHandle);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoMpu9250ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	uint8_t tmpBuffer[1];
	mlsErrorCode_t errorCode = MLS_ERROR;

	tmpBuffer[0] = regAddr;
	errorCode = HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDR, tmpBuffer, 1, 100);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	errorCode = HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDR, buffer, len, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoMpu9250WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	uint8_t bufferSend[len+1];

	bufferSend[0] = regAddr;
	for(uint8_t i = 0; i < len; i++)
	{
		bufferSend[i + 1] = buffer[i];
	}
	errorCode = HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDR, bufferSend, len + 1, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoAk8963ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	uint8_t tmpBuffer[1];
	mlsErrorCode_t errorCode = MLS_ERROR;

	tmpBuffer[0] = regAddr;
	errorCode = HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, tmpBuffer, 1, 100);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	errorCode = HAL_I2C_Master_Receive(&hi2c1, AK8963_ADDRESS, buffer, len, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoAk8963WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	uint8_t bufferSend[len+1];

	bufferSend[0] = regAddr;
	for(uint8_t i = 0; i < len; i++)
	{
		bufferSend[i + 1] = buffer[i];
	}
	errorCode = HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, bufferSend, len + 1, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoUartReadBytes(uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = HAL_UART_Receive_DMA(&huart2, buffer, len);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoUartWriteBytes(uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = HAL_UART_Transmit_DMA(&huart2, buffer, len);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorSetDuty(float duty)
{
	/* Calculate PWM compare value */
	uint32_t computeValue;
	computeValue = duty * (HW_LEFTMOTOR_TIM_HANDLE.Instance->ARR) / 100;

	/* Configure PWM compare value */
	__HAL_TIM_SET_COMPARE(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL, computeValue);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorSetFrequency(uint32_t freq)
{
	if (freq == 0)
	{
		__HAL_TIM_SET_AUTORELOAD(&HW_LEFTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_PRESCALER(&HW_LEFTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_COMPARE(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL, 0);

		return MLS_SUCCESS;
	}

	/* Calculate Timer PWM parameters. When change timer period you also
	 * need to update timer compare value to keep duty cycle stable */
	uint32_t apbFreq = HW_LEFTMOTOR_TIM_CLK_FREQ;
	uint32_t conduct = (uint32_t)(apbFreq / freq);
	uint16_t timerPrescaler = conduct / TIMER_MAX_RELOAD + 1;
	uint16_t timerPeriod = (uint16_t)(conduct / (timerPrescaler + 1)) - 1;

	__HAL_TIM_SET_AUTORELOAD(&HW_LEFTMOTOR_TIM_HANDLE, timerPeriod);
	__HAL_TIM_SET_PRESCALER(&HW_LEFTMOTOR_TIM_HANDLE, timerPrescaler);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorStart(void)
{
	HAL_TIM_PWM_Start(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorStop(void)
{
	HAL_TIM_PWM_Stop(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorSetDir(uint8_t dir)
{
	HAL_GPIO_WritePin(HW_LEFTMOTOR_GPIO, HW_LEFTMOTOR_GPIO_PIN, dir);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorSetDuty(float duty)
{
	/* Calculate PWM compare value */
	uint32_t computeValue;
	computeValue = duty * (HW_RIGHTMOTOR_TIM_HANDLE.Instance->ARR) / 100;

	/* Configure PWM compare value */
	__HAL_TIM_SET_COMPARE(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL, computeValue);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorSetFrequency(uint32_t freq)
{
	if (freq == 0)
	{
		__HAL_TIM_SET_AUTORELOAD(&HW_RIGHTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_PRESCALER(&HW_RIGHTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_COMPARE(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL, 0);

		return MLS_SUCCESS;
	}

	/* Calculate Timer PWM parameters. When change timer period you also
	 * need to update timer compare value to keep duty cycle stable */
	uint32_t apbFreq = HW_RIGHTMOTOR_TIM_CLK_FREQ;
	uint32_t conduct = (uint32_t)(apbFreq / freq);
	uint16_t timerPrescaler = conduct / TIMER_MAX_RELOAD + 1;
	uint16_t timerPeriod = (uint16_t)(conduct / (timerPrescaler + 1)) - 1;

	__HAL_TIM_SET_AUTORELOAD(&HW_RIGHTMOTOR_TIM_HANDLE, timerPeriod);
	__HAL_TIM_SET_PRESCALER(&HW_RIGHTMOTOR_TIM_HANDLE, timerPrescaler);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorStart(void)
{
	HAL_TIM_PWM_Start(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorStop(void)
{
	HAL_TIM_PWM_Stop(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorSetDir(uint8_t dir)
{
	HAL_GPIO_WritePin(HW_RIGHTMOTOR_GPIO, HW_RIGHTMOTOR_GPIO_PIN, dir);

	return MLS_SUCCESS;
}
/**@}*/
