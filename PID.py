import cv2
import numpy as np
from ultralytics import YOLO
import time
from pid_controller import PIDController
from kalman_filter import KalmanFilter1D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

# Global PID yönelme çıktıları
def update_pid_output(yaw, pitch):
    global current_yaw_output, current_pitch_output
    current_yaw_output = yaw
    current_pitch_output = pitch

current_yaw_output = 0.0
current_pitch_output = 0.0

# Yön oku plotu (merkezden PID yönüne ok)
def directional_plot():
    fig, ax = plt.subplots()
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title("PID Yönelme Vektörü")
    ax.set_xlabel("YAW (Sağ/Sol)")
    ax.set_ylabel("PITCH (Yukarı/Aşağı)")
    ax.plot(0, 0, 'ko', markersize=8)

    arrow = ax.arrow(0, 0, 0, 0, head_width=5, head_length=10, fc='r', ec='r')

    def animate(frame):
        nonlocal arrow
        arrow.remove()
        # Reverse yön: -yaw, +pitch
        arrow = ax.arrow(0, 0, -current_yaw_output, current_pitch_output,
                         head_width=5, head_length=10, fc='r', ec='r')
        return arrow,

    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.tight_layout()
    plt.show()

# Thread başlat
threading.Thread(target=directional_plot, daemon=True).start()

# PID Çıkış Kaydı
def log_pid_outputs(yaw, pitch):
    update_pid_output(yaw, pitch)

# YOLOv8 modelini yükle
model = YOLO(r"C:\\Users\\bxnxa\\Desktop\\yolov8obb\\runs\\obb\\train8\\weights\\best.pt")
video_path = r"C:\\Users/bxnxa\\Desktop\\yolov8obb\\video.mp4"
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Video açılamadı, dosya yolunu kontrol edin!", video_path)
    exit()

class_names = {0: "planes"}
detection_counter = 0
successful_detection_time = None
is_successful = False
success_display_time = None
max_detection_time = 4

pid_x = PIDController(Kp=0.2, Ki=0.01, Kd=0.1, setpoint=320)
kalman_x = KalmanFilter1D()
pid_y = PIDController(Kp=0.2, Ki=0.01, Kd=0.1, setpoint=240)
kalman_y = KalmanFilter1D()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2
    pid_x.setpoint = center_x
    pid_y.setpoint = center_y

    top = int(height * 0.1)
    bottom = int(height * 0.9)
    left = int(width * 0.25)
    right = int(width * 0.75)
    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

    results = model(frame)
    detected_target = False

    for r in results:
        boxes = r.boxes.xyxy.cpu().numpy()
        confidences = r.boxes.conf.cpu().numpy()
        classes = r.boxes.cls.cpu().numpy()

        for box, conf, cls in zip(boxes, confidences, classes):
            if conf > 0.5 and cls in class_names:
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

                box_center_x = (x1 + x2) // 2
                box_center_y = (y1 + y2) // 2

                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.arrowedLine(frame, (box_center_x, box_center_y), (center_x, center_y), (255, 0, 0), 2)

                label = f"{class_names[int(cls)]} {conf:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                if left <= box_center_x <= right and top <= box_center_y <= bottom:
                    detected_target = True

                kalman_x.predict()
                kalman_x.update(box_center_x)
                filtered_x = kalman_x.get_estimate()
                kalman_y.predict()
                kalman_y.update(box_center_y)
                filtered_y = kalman_y.get_estimate()

                dt = 1 / 30
                yaw_output = pid_x.update(filtered_x, dt)
                pitch_output = pid_y.update(filtered_y, dt)

                print(f"X: {box_center_x:.1f} | X_Kalman: {filtered_x:.1f} | PID_YAW: {yaw_output:.2f} || ",
                      f"Y: {box_center_y:.1f} | Y_Kalman: {filtered_y:.1f} | PID_PITCH: {pitch_output:.2f}")

                log_pid_outputs(yaw_output, pitch_output)

    if detected_target:
        if successful_detection_time is None:
            successful_detection_time = time.time()
    else:
        successful_detection_time = None

    if successful_detection_time and time.time() - successful_detection_time >= max_detection_time:
        is_successful = True

    if is_successful:
        if success_display_time is None:
            success_display_time = time.time()
        cv2.putText(frame, "Locked", (width // 2 - 50, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)

    if success_display_time and time.time() - success_display_time >= 2:
        is_successful = False
        success_display_time = None

    if successful_detection_time:
        elapsed_time = time.time() - successful_detection_time
        if elapsed_time <= max_detection_time:
            cv2.putText(frame, f"{elapsed_time:.2f}s", (left, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        else:
            cv2.putText(frame, f"{max_detection_time}s", (left, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

    cv2.imshow("YOLOv8 Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
