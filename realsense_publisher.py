import os
import sys

# NUCLEAR WINDOWS FIX: Hardcode environment before ANYTHING else loads
PIXI_ENV = r"C:\pixi_ws\.pixi\envs\default"
os.environ["AMENT_PREFIX_PATH"] = PIXI_ENV
os.environ["ROS_VERSION"] = "2"
os.environ["ROS_DOMAIN_ID"] = "0"
# Force PIXI bin to the front of PATH for DLL resolution
pixi_bin = os.path.join(PIXI_ENV, "Library", "bin")
if pixi_bin not in os.environ.get("PATH", ""):
    os.environ["PATH"] = f"{pixi_bin};{os.environ.get('PATH', '')}"

import pyrealsense2 as rs
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

print("--- Camera Publisher Script Started ---")

class UniversalCameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        
        self.use_realsense = False
        self.cap = None
        self.pipeline = None
        self.use_dummy = False
        
        # Try RealSense First
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            try:
                self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            except Exception:
                self.get_logger().warn("RealSense depth stream not available")
                
            self.pipeline.start(self.config)
            self.use_realsense = True
            self.get_logger().info('RealSense Camera connected!')
        except Exception as e:
            self.get_logger().warn(f'RealSense SDK failed ({e}). Trying fallback indices...')
            self.use_realsense = False
            
        # Fallback to OpenCV
        if not self.use_realsense:
            for i in range(5):
                self.cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
                if self.cap and self.cap.isOpened():
                    self.get_logger().info(f'Standard Webcam connected on index {i}!')
                    break
                else:
                    if self.cap: self.cap.release()
                    self.cap = None
                
            if not self.cap or not self.cap.isOpened():
                self.get_logger().error('No camera found (neither RealSense nor Webcam indices 0-4)!')
                self.use_dummy = True
                self.get_logger().info('Using DUMMY black images (camera simulation active)')
            else:
                self.use_dummy = False

        self.timer = self.create_timer(1.0/15.0, self.timer_callback)

    def timer_callback(self):
        color_image = None
        depth_image = None
        
        if self.use_realsense:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                else:
                    depth_image = np.zeros((480, 640), dtype=np.uint16)
            except Exception as e:
                self.get_logger().warn(f"RS Frame error: {e}")
                
        elif not self.use_dummy and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                color_image = frame
                depth_image = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint16)
        
        elif self.use_dummy:
            color_image = np.zeros((480, 640, 3), dtype=np.uint8)
            depth_image = np.zeros((480, 640), dtype=np.uint16)
            cv2.putText(color_image, "DUMMY CAMERA - NO HARDWARE detected", (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(color_image, "Check connections and restart", (50, 280), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        if color_image is not None:
             # Publish Color
            msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_color_optical_frame"
            self.color_pub.publish(msg)
            
            # Publish Depth
            if depth_image is not None:
                d_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                d_msg.header.stamp = self.get_clock().now().to_msg()
                d_msg.header.frame_id = "camera_depth_optical_frame"
                self.depth_pub.publish(d_msg)

    def __del__(self):
        if self.pipeline: 
            try: self.pipeline.stop()
            except: pass
        if self.cap: self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = UniversalCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
