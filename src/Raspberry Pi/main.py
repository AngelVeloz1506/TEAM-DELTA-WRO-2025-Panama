# -- coding: utf-8 --
# Color detection with HSV trackbars (red and green tuning) - OpenCV

import cv2
import numpy as np
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()

def open_camera(index=0, width=640, height=480):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap

cap = open_camera(0, 1080, 640)
kernel = np.ones((5, 5), np.uint8)

# --- Trackbar initialization ---
def nothing(x): pass

cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Controls", 500, 400)

# Red HSV ranges
cv2.createTrackbar("R_lowH", "Controls", 0,   179, nothing)
cv2.createTrackbar("R_highH", "Controls", 5, 179, nothing)
cv2.createTrackbar("R_lowS", "Controls", 110, 255, nothing)
cv2.createTrackbar("R_highS", "Controls", 255, 255, nothing)
cv2.createTrackbar("R_lowV", "Controls", 20,  255, nothing)
cv2.createTrackbar("R_highV", "Controls", 255, 255, nothing)

# Green HSV ranges
cv2.createTrackbar("G_lowH", "Controls", 35, 179, nothing)
cv2.createTrackbar("G_highH", "Controls", 85, 179, nothing)
cv2.createTrackbar("G_lowS", "Controls", 80, 255, nothing)
cv2.createTrackbar("G_highS", "Controls", 255, 255, nothing)
cv2.createTrackbar("G_lowV", "Controls", 70, 255, nothing)
cv2.createTrackbar("G_highV", "Controls", 255, 255, nothing)

def color_mask(hsv, low, high):
    return cv2.inRange(hsv, low, high)

def top_blobs(mask, max_n=3, min_area=800):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return [], mask
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

    # --- Read trackbar values dynamically ---
    r_lh = cv2.getTrackbarPos("R_lowH", "Controls")
    r_hh = cv2.getTrackbarPos("R_highH", "Controls")
    r_ls = cv2.getTrackbarPos("R_lowS", "Controls")
    r_hs = cv2.getTrackbarPos("R_highS", "Controls")
    r_lv = cv2.getTrackbarPos("R_lowV", "Controls")
    r_hv = cv2.getTrackbarPos("R_highV", "Controls")

    g_lh = cv2.getTrackbarPos("G_lowH", "Controls")
    g_hh = cv2.getTrackbarPos("G_highH", "Controls")
    g_ls = cv2.getTrackbarPos("G_lowS", "Controls")
    g_hs = cv2.getTrackbarPos("G_highS", "Controls")
    g_lv = cv2.getTrackbarPos("G_lowV", "Controls")
    g_hv = cv2.getTrackbarPos("G_highV", "Controls")

    # --- Build masks ---
    lower_red = np.array([r_lh, r_ls, r_lv])
    upper_red = np.array([r_hh, r_hs, r_hv])
    mask_red = color_mask(hsv, lower_red, upper_red)

    lower_green = np.array([g_lh, g_ls, g_lv])
    upper_green = np.array([g_hh, g_hs, g_hv])
    mask_green = color_mask(hsv, lower_green, upper_green)

    detections = []

    cnts_r, _ = top_blobs(mask_red, max_n=3, min_area=800)
    for c in cnts_r:
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        detections.append((1, area, (x, y, w, h), (0, 0, 255)))

    cnts_g, _ = top_blobs(mask_green, max_n=3, min_area=800)
    for c in cnts_g:
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        detections.append((0, area, (x, y, w, h), (0, 255, 0)))

    # --- Sort and pick only the largest area (closest object) ---
    if detections:
        detections.sort(key=lambda x: x[1], reverse=True)
        name, area, (x, y, w, h), bgr = detections[0]

        out = frame.copy()
        cv2.rectangle(out, (x, y), (x + w, y + h), bgr, 2)
        cv2.putText(out, f"{name} ({int(area)})", (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 2, cv2.LINE_AA)

        print(f"{name}--{x:03d}--{w:03d}")
        ser.write(f'{name}{x:03d}{w:03d}\n'.encode('utf-8'))
    else:
        out = frame.copy()
        print("No object detected")
        ser.write('NOBJ\n'.encode('utf-8'))

    cv2.imshow("color_detection", out)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('q') or k == 27:
        break

cap.release()
cv2.destroyAllWindows()
ser.close()