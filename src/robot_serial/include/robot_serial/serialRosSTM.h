#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <string.h>
#include <thread>

#define SERIAL_SOF  0xAA    // Start of Frame
#define SERIAL_EOF  0xBB    // End of Frame
#define SERIAL_ACK  0x00    // ACK
#define SERIAL_NACK 0x01    // NACK

#define SERIAL_MAX_LENGTH		1024

typedef enum : uint8_t{
	SERIAL_MODE_IMU 	= 0x00,
	SERIAL_MODE_WHEEL	= 0x01,
	SERIAL_MODE_CMD_VEL	= 0X02
} serialMode_t;

typedef struct imu_vec3 {
    double x;
    double y;
    double z;
} imu_vec3_t;

typedef struct imu_vec4 {
	double x;
	double y;
	double z;
	double w;
} imu_vec4_t;

typedef struct imu_euler {
	double roll;
	double pitch;
	double yaw;
} imu_euler_t;

typedef struct __attribute__((packed)){
	uint8_t 		header;								/*!< Header of data frame */
	uint16_t 		length;								/*!< Length of data (exclude header, length, mode, CRC, footer)*/
	serialMode_t	mode;								/*!< Mode */
	uint32_t  		crc;								/*!< Check sum */
	uint8_t			dataBuff[SERIAL_MAX_LENGTH - 9];	/*!< Data buffer */
	uint8_t			footer;								/*!< Footer of data frame */
} dataFrame_t;

typedef struct __attribute__((packed)){
	imu_vec3_t		accel;
	imu_vec3_t		gyro;
	imu_vec3_t		mag;
	imu_vec4_t		quaternion;
	imu_euler_t		euler;
} imu_data_t;

typedef struct {
	double 		leftWheel;
	double 		rightWheel;
} wheel_velocity_t;

class SerialNode {
    public:
        SerialNode(ros::NodeHandle& nh, const std::string& port, uint32_t baud_rate);
        ~SerialNode();
    
        uint32_t calculateCRC(uint8_t *data, uint16_t length);
    
    private:
		ros::NodeHandle nh_;
		ros::Publisher pub_;
		ros::Subscriber sub_;
		serial::Serial serial_port_;
		std::thread readThread_;
		sensor_msgs::Imu imuDataMsg;
    
        void readData();
    
        void sendDataToSTM32(const std_msgs::String::ConstPtr& msg);
};