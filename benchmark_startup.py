import os
import sys
import time

# NUCLEAR WINDOWS FIX: Hardcode environment before ANYTHING else loads
PIXI_ENV = r"C:\pixi_ws\.pixi\envs\default"
os.environ["AMENT_PREFIX_PATH"] = PIXI_ENV
os.environ["ROS_VERSION"] = "2"
os.environ["ROS_DOMAIN_ID"] = "0"
# Force PIXI bin to the front of PATH for DLL resolution
pixi_bin = os.path.join(PIXI_ENV, "Library", "bin")
if pixi_bin not in os.environ.get("PATH", ""):
    os.environ["PATH"] = f"{pixi_bin};{os.environ.get('PATH', '')}"

def benchmark():
    results = []
    
    t0 = time.time()
    try:
        import rclpy
        results.append(f"rclpy import: {time.time()-t0:.4f}s")
        
        t0 = time.time()
        # Try a full node initialization
        rclpy.init()
        results.append(f"rclpy.init(): {time.time()-t0:.4f}s")
        
        node = rclpy.create_node('benchmark_node')
        results.append(f"node creation: {time.time()-t0:.4f}s")
        
        t0 = time.time()
        topics = node.get_topic_names_and_types()
        results.append(f"topic list: {time.time()-t0:.4f}s")
        results.append(f"Count: {len(topics)}")
        
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        results.append(f"Failure: {e}")

    print("\n".join(results))

if __name__ == "__main__":
    benchmark()
