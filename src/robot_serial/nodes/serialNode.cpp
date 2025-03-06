#include "../include/robot_serial/serialRosSTM.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "stm32_serial_node");
    ros::NodeHandle private_nh("~");
    std::string serial_port;
    int baud_rate;
    private_nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    private_nh.param<int>("baud_rate", baud_rate, 115200);

    SerialNode serial_node(private_nh, serial_port, baud_rate);
    ros::spin();
    return 0;
}