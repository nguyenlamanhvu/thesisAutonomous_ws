#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def qr_code_callback(data):
    rospy.loginfo(f"Nhận được thông báo: {data.data}")
    # gui hien thi xu ly o day

def gui_server():
    rospy.init_node('gui_server', anonymous=True)
    rospy.Subscriber("/qr_code_result", String, qr_code_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        gui_server()
    except rospy.ROSInterruptException:
        pass