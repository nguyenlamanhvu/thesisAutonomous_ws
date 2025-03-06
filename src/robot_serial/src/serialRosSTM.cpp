#include "../include/robot_serial/serialRosSTM.h"


SerialNode::SerialNode(ros::NodeHandle& nh, const std::string& port, uint32_t baud_rate)
    : nh_(nh), serial_port_(port, baud_rate, serial::Timeout::simpleTimeout(100)) {

    pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
    sub_ = nh_.subscribe("send_to_stm32", 10, &SerialNode::sendDataToSTM32, this);

    if (serial_port_.isOpen()) {
        ROS_INFO("Serial port %s opened at baud rate %d", port.c_str(), baud_rate);
    } else {
        ROS_ERROR("Failed to open serial port %s", port.c_str());
    }

    readThread_ = std::thread(&SerialNode::readData, this);
}

SerialNode::~SerialNode() {
    serial_port_.close();
    if (readThread_.joinable()) {
        readThread_.join();
    }
}

uint32_t SerialNode::calculateCRC(uint8_t *data, uint16_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 * (crc & 1));
        }
    }
    return ~crc;
}


void SerialNode::readData() {
    // while (ros::ok()) {
    //     if (serial_port_.available() >= sizeof(dataFrame_t)) {
    //         std::vector<uint8_t> buffer(sizeof(dataFrame_t));
    //         serial_port_.read(buffer, sizeof(dataFrame_t));

    //         dataFrame_t* frame = reinterpret_cast<dataFrame_t*>(buffer.data());

    //         if (frame->header == SERIAL_SOF && frame->footer == SERIAL_EOF) {
    //             uint32_t crc_check = calculateCRC(buffer.data(), frame->length + 3);
    //             if (crc_check == frame->crc) {
    //                 imuDataMsg.header.frame_id = "/map";
    //                 imuDataMsg.header.stamp = ros::Time::now();

    //                 imu_data_t* data = reinterpret_cast<imu_data_t*>(frame->dataBuff);
    //                 imuDataMsg.linear_acceleration.x = data->accel.x;
    //                 imuDataMsg.linear_acceleration.y = data->accel.y;
    //                 imuDataMsg.linear_acceleration.z = data->accel.z;

    //                 imuDataMsg.angular_velocity.x = data->gyro.x;
    //                 imuDataMsg.angular_velocity.y = data->gyro.y;
    //                 imuDataMsg.angular_velocity.z = data->gyro.z;

    //                 imuDataMsg.orientation.w = data->quaternion.w;
    //                 imuDataMsg.orientation.x = data->quaternion.x;
    //                 imuDataMsg.orientation.y = data->quaternion.y;
    //                 imuDataMsg.orientation.z = data->quaternion.z;
    //                 pub_.publish(imuDataMsg);

    //                 ROS_INFO("Received Valid IMU Data");
    //             } else {
    //                 ROS_ERROR("CRC ERROR");
    //             }
    //         }
    //     }
    //     ros::Duration(0.01).sleep();  // Avoid busy loop
    //     ROS_INFO("Received IMU Data avai %d size %d", serial_port_.available(), sizeof(dataFrame_t));
    // }

    while (ros::ok())
    {
        static std::vector<uint8_t> recvBuffer;
        size_t available = serial_port_.available();
        if (available > 0) {
            std::vector<uint8_t> tempBuffer(available);
            serial_port_.read(tempBuffer, available);
            recvBuffer.insert(recvBuffer.end(), tempBuffer.begin(), tempBuffer.end());
            ROS_INFO("Received IMU Data avai %d", recvBuffer.size());
        }
    
        while (recvBuffer.size() >= sizeof(dataFrame_t)) {
            dataFrame_t* frame = reinterpret_cast<dataFrame_t*>(recvBuffer.data());
    
            if (frame->header == SERIAL_SOF && frame->footer == SERIAL_EOF) {
                uint32_t crc_check = calculateCRC(recvBuffer.data(), frame->length + 3);
                if (crc_check == frame->crc) {
                    imuDataMsg.header.frame_id = "/map";
                    imuDataMsg.header.stamp = ros::Time::now();
    
                    imu_data_t* data = reinterpret_cast<imu_data_t*>(frame->dataBuff);
                    imuDataMsg.linear_acceleration.x = data->accel.x;
                    imuDataMsg.linear_acceleration.y = data->accel.y;
                    imuDataMsg.linear_acceleration.z = data->accel.z;
    
                    imuDataMsg.angular_velocity.x = data->gyro.x;
                    imuDataMsg.angular_velocity.y = data->gyro.y;
                    imuDataMsg.angular_velocity.z = data->gyro.z;
    
                    imuDataMsg.orientation.w = data->quaternion.w;
                    imuDataMsg.orientation.x = data->quaternion.x;
                    imuDataMsg.orientation.y = data->quaternion.y;
                    imuDataMsg.orientation.z = data->quaternion.z;
                    pub_.publish(imuDataMsg);
    
                    pub_.publish(imuDataMsg);
                    ROS_INFO("Received Valid IMU Data");
                } else {
                    ROS_ERROR("CRC ERROR");
                }
    
                // Remove processed frame from buffer
                recvBuffer.erase(recvBuffer.begin(), recvBuffer.begin() + sizeof(dataFrame_t));
            } else {
                recvBuffer.erase(recvBuffer.begin()); // Discard bad header
            }
           
        }
    }
}

void SerialNode::sendDataToSTM32(const std_msgs::String::ConstPtr& msg) {
    std::string data = msg->data;
    uint8_t txBuffer[sizeof(dataFrame_t)];
    dataFrame_t* frame = reinterpret_cast<dataFrame_t*>(txBuffer);

    frame->header = SERIAL_SOF;
    frame->length = data.size();
    memcpy(frame->dataBuff, data.c_str(), data.size());
    frame->crc = calculateCRC(reinterpret_cast<uint8_t*>(frame), frame->length + 3);
    frame->footer = SERIAL_EOF;

    serial_port_.write(txBuffer, sizeof(dataFrame_t));
    ROS_INFO("Sent data to STM32: %s", msg->data.c_str());
}
