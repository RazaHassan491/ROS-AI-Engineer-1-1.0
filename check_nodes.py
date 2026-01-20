import os
import sys
import rclpy

# NUCLEAR WINDOWS FIX
PIXI_ENV = r"C:\pixi_ws\.pixi\envs\default"
os.environ["AMENT_PREFIX_PATH"] = PIXI_ENV
os.environ["ROS_VERSION"] = "2"
os.environ["ROS_DOMAIN_ID"] = "0"
pixi_bin = os.path.join(PIXI_ENV, "Library", "bin")
os.environ["PATH"] = f"{pixi_bin};{os.environ.get('PATH', '')}"

def check():
    rclpy.init()
    node = rclpy.create_node("check_nodes_node")
    print("Nodes found:")
    for n in node.get_node_names():
        print(f" - {n}")
    
    print("\nTopics found:")
    for name, types in node.get_topic_names_and_types():
        print(f" - {name} [{', '.join(types)}]")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    check()
