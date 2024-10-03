#!/usr/bin/env python3

import cv2
import rospy
from std_msgs.msg import String

class QRCodeScanner:
    def __init__(self, camera_index=0):
        rospy.init_node('qr_code_scanner', anonymous=True)
        self.qr_pub = rospy.Publisher('/qr_code_result', String, queue_size=10)
        
        # tao doi tuong qr code
        self.qr_code_detector = cv2.QRCodeDetector()
        
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            rospy.loginfo("Không thể mở camera")
            exit()

    def detect_and_publish(self):
        #quet qr va publish thong bao
        while not rospy.is_shutdown():
            # Đọc khung hình từ camera
            ret, frame = self.cap.read()
            if not ret:
                rospy.loginfo("Không thể nhận diện khung hình")
                break

            # phat hien va giai ma qr
            data, bbox, _ = self.qr_code_detector.detectAndDecode(frame)
            
            if bbox is not None:
                # lam 1 khung xung quang QR de nhan dien
                for i in range(len(bbox)):
                    point1 = tuple(bbox[i][0])
                    point2 = tuple(bbox[(i + 1) % len(bbox)][0])
                    cv2.line(frame, point1, point2, color=(255, 0, 0), thickness=2)
                
                if data:
                    #  hien thi noi dung
                    cv2.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    rospy.loginfo(f"QR Code: {data}")

                    self.qr_pub.publish(f"QR Code detected: {data}")

            cv2.imshow("QR Code Scanner", frame)
    
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def release_resources(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        # quan ly chay truong trinh
        qr_scanner = QRCodeScanner(camera_index=2)
        qr_scanner.detect_and_publish()
    except rospy.ROSInterruptException:
        pass
    finally:
        qr_scanner.release_resources()
