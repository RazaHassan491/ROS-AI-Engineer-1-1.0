import os
import sys
import datetime

# --- EMERGENCY LOGGING ---
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "capture_life_sign.log")
def log_step(msg):
    with open(LOG_FILE, "a") as f:
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        f.write(f"[{timestamp}] {msg}\n")
        print(f"[{timestamp}] {msg}")

log_step("--- CAPTURE SCRIPT STARTING ---")

try:
    log_step("Attempting to import rclpy...")
    import rclpy
    log_step("Imported rclpy successfully.")
    
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    log_step("All dependencies imported.")
except Exception as e:
    log_step(f"CRITICAL IMPORT ERROR: {e}")
    sys.exit(1)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_emergency')
        self.bridge = CvBridge()
        self.color_received = False
        self.depth_received = False

        log_step("Creating subscriptions...")
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            qos_profile=qos_profile_sensor_data)

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            qos_profile=qos_profile_sensor_data)
        
        log_step("Subscriptions created. Waiting for images...")
        self.timer = self.create_timer(40.0, self.timer_callback)

    def timer_callback(self):
        log_step("TIMED OUT: No images received on topics.")
        self.destroy_node()
        rclpy.shutdown()

    def color_callback(self, msg):
        if not self.color_received:
            log_step("Color image received!")
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                os.makedirs('camera_images', exist_ok=True)
                cv2.imwrite(os.path.join('camera_images', 'color_image.png'), cv_image)
                self.color_received = True
                self.check_completion()
            except Exception as e:
                log_step(f"Color save error: {e}")

    def depth_callback(self, msg):
        if not self.depth_received:
            log_step("Depth image received!")
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                os.makedirs('camera_images', exist_ok=True)
                cv2.imwrite(os.path.join('camera_images', 'depth_image.png'), cv_image)
                self.depth_received = True
                self.check_completion()
            except Exception as e:
                log_step(f"Depth save error: {e}")

    def check_completion(self):
        if self.color_received and self.depth_received:
            log_step("SUCCESS: Both images saved.")
            self.destroy_node()
            rclpy.shutdown()

def main():
    log_step("Initializing rclpy...")
    try:
        rclpy.init()
        node = ImageSubscriber()
        log_step("Spinning node...")
        rclpy.spin(node)
    except Exception as e:
        log_step(f"Runtime error: {e}")
    finally:
        log_step("Script exiting.")

if __name__ == '__main__':
    main()
