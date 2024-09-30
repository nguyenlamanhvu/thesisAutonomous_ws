import cv2
import numpy as np

cap = cv2.VideoCapture(2)  # nếu có nhiều cam thì thêm id webcam 1, 2, 3...
# cap = cv2.VideoCapture("1.mp4") 
# nếu không có webcam có thể thay bằng video name

while True:
    ret, frame = cap.read()
    width = int(cap.get(3))
    height = int(cap.get(4))

    small_frame = cv2.resize(frame,(0,0),fx=0.5,fy=0.5)

    image = np.zeros(frame.shape,np.uint8)

    image[:height//2,:width//2]=small_frame
    image[:height//2,width//2:]=small_frame
    image[height//2:,:width//2]=small_frame
    image[height//2:,width//2:]=small_frame

    
    #print(ret)  
    # ret sẽ trả về True/False,
    # True khi quá trình chụp ảnh diễn ra ok
    # False khi camera bị chiếm dụng bởi phần mềm khác

    cv2.imshow("Camera Show", image)
    
    if cv2.waitKey(1) == ord("q"):
        # đợi 1 mili giây, nhấn q để thoát
        break

cap.release()  # giải phóng camera
cv2.destroyAllWindows()  # exit


