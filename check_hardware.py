import pyrealsense2 as rs
import sys

def check_realsense():
    print("Checking for RealSense devices...")
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            print("No RealSense devices found.")
            return False
        
        for i, dev in enumerate(devices):
            print(f"Device {i}: {dev.get_info(rs.camera_info.name)} (S/N: {dev.get_info(rs.camera_info.serial_number)})")
        return True
    except Exception as e:
        print(f"Error checking RealSense: {e}")
        return False

if __name__ == "__main__":
    check_realsense()
