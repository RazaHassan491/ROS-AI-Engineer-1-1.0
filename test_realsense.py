import pyrealsense2 as rs
import cv2
import sys

print("--- Testing RealSense SDK ---")
try:
    ctx = rs.context()
    devices = ctx.query_devices()
    print(f"DEBUG: Found {len(devices)} RealSense devices")
    for d in devices:
        print(f"DEBUG: Device Name: {d.get_info(rs.camera_info.name)}")
except Exception as e:
    print(f"SDK Error: {e}")

print("\n--- Testing OpenCV Fallback ---")
for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"SUCCESS: Found functional camera on Index {i} (Shape: {frame.shape})")
            cap.release()
        else:
            print(f"Index {i}: Opened but failed to read frame")
            cap.release()
    else:
        # print(f"Index {i}: Not opened")
        pass
