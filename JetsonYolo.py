#!/usr/bin/env python3
"""
Deteksi cabai matang dengan YOLOv5 TensorRT + kontrol PID + inverse kinematics
Perbaikan:
 - PID stabil (anti-windup, derivative berbasis dt)
 - Handling serial/dry-run
 - Proteksi arccos domain error
 - Clamp servo ke 0..1023
"""

import sys
import cv2
import serial
import imutils
import numpy as np
import time
import math
from yoloDet import YoloTRT
#no_object = False
# === Load model TensorRT ===
model = YoloTRT(
    library="yolov5/build/libmyplugins.so",
    engine="yolov5/build/cabecabean.engine",
    conf=0.5,
    yolo_ver="v5"
)
# --- Serial ke Arduino ---
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200
arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
time.sleep(2)
XT, YT = 320, 240   # target center pixel
# --- Kamera ---
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    frame = imutils.resize(frame, width=600)

    detections, t = model.Inference(frame)
    if len(detections) == 0:
        #if not no_object:
        msg = f"{2000},{2000},{2000}\n"
        arduino.write(msg.encode())
        cv2.putText(frame, "Cabai tidak terdeteksi", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

    found = False
    for obj in detections:
        cls = obj['class']
        conf = obj['conf']
        x1, y1, x2, y2 = obj['box']

        if cls != "Matang": # target hanya cabai matang
            continue

        found = True
        x_center = int((x1+x2)/2)
        y_center = int((y1+y2)/2)
        w, h = x2-x1, y2-y1
        jarak = (2250/w) - 20

        # draw box
        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.circle(frame, (x_center,y_center), 4, (255,0,0), -1)

        # ERROR
        ex, ey = XT-x_center, YT-y_center
        print(f"[INFO] Servo: x={ex}, y={ey}, jarak={jarak}")
        if arduino.in_waiting > 0 :
            info = arduino.readline().decode().strip()
            print("Arduino", info)

        # kirim serial
        msg = f"{ex},{ey},{jarak}\n"
        arduino.write(msg.encode())
        break
        # Tracking arah
        frame_center_x = frame.shape[1] // 2
        offset = x_center - frame_center_x
        if abs(offset) > 30:
            print("Objek menyimpang, tracking arah...")
            break

        print("End-effector memotong cabai...")
        time.sleep(2)
        print("Reset ke posisi awal...")
        time.sleep(1)
        break
    if not found:
        msg = f"{2000},{2000},{2000}\n"
        arduino.write(msg.encode())
        cv2.putText(frame, "Tidak ada cabai matang valid", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        

    cv2.imshow("Deteksi YOLOv5", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
if arduino: arduino.close()
cv2.destroyAllWindows()
