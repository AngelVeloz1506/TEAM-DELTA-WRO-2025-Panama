# -- coding: utf-8 --
# Color detection with OpenCV (ASCII-only version) - multi targets, no BLUE, broader RED

import cv2
import numpy as np
import serial
import time
#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
#time.sleep(3)
#ser.reset_input_buffer()

def open_camera(index=0, width=640, height=480):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap

cap = open_camera(0, 800, 600)
# HSV ranges (OpenCV: H=0..179, S=0..255, V=0..255)
# Red broadened to capture darker/mid reds
RED_1  = (np.array([0,   110,  20]),  np.array([10,  255, 255]))
RED_2  = (np.array([170, 110,  20]),  np.array([10, 255, 255]))

# Green as before
GREEN  = (np.array([35,  80,  70]),   np.array([85,  255, 255]))

kernel = np.ones((5, 5), np.uint8)

def color_mask(hsv, rng):
    low, high = rng
    return cv2.inRange(hsv, low, high)

def top_blobs(mask, max_n=3, min_area=800):
    # Morphological cleaning
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return [], mask
    # Filter and sort
    cnts = [c for c in cnts if cv2.contourArea(c) >= min_area]
    cnts.sort(key=cv2.contourArea, reverse=True)
    return cnts[:max_n], mask

while True:
    ok, frame = cap.read()
    if not ok:
        print("Camera read failed.")
        break

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Masks (NO BLUE)
    m_r1 = color_mask(hsv, RED_1)
    m_r2 = color_mask(hsv, RED_2)
    mask_red = cv2.bitwise_or(m_r1, m_r2)
    mask_green = color_mask(hsv, GREEN)

    # Collect detections (max 3 total)
    detections = []

    cnts_r, _ = top_blobs(mask_red, max_n=3, min_area=800)
    for c in cnts_r:
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        detections.append((1, area, (x, y, w, h), (0, 0, 255)))#RED

    cnts_g, _ = top_blobs(mask_green, max_n=3, min_area=800)
    for c in cnts_g:
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        detections.append((0, area, (x, y, w, h), (0, 255, 0)))#GREEN

    # Sort by area, keep top 3
    detections.sort(key=lambda x: x[1], reverse=True)
    detections = detections[:3]

    out = frame.copy()
    # Draw detections
    for idx, (name, area, (x, y, w, h), bgr) in enumerate(detections, start=1):
        cv2.rectangle(out, (x, y), (x + w, y + h), bgr, 2)
        cv2.putText(out, f"{idx}:{name} ({int(area)})", (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 2, cv2.LINE_AA)
        print(f"{name}--{x:03d}--{w:03d}")
        #ser.write(f'{name}{x:03d}{w:03d}\n'.encode('utf-8'))
    cv2.imshow("color_detection", out)
    #cv2.imshow("mask_red", mask_red)
    #cv2.imshow("mask_green", mask_green)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('q') or k == 27:
        break

cap.release()
cv2.destroyAllWindows()
#ser.close