import cv2 as cv
import numpy as np

import time

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

detector = cv.QRCodeDetector()


while cap.isOpened():

    success, img = cap.read()
    