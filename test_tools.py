import sys
import os

# Explicitly add the project src to the front of sys.path
project_src = r"c:\Users\Raza Hassan\Downloads\ROS-AI-Engineer-1.0-main\ROS-AI-Engineer-1.0-main\ROS-AI-Engineer-1.0-main\src"
if project_src not in sys.path:
    sys.path.insert(0, project_src)

# Add pixi bin to PATH for subprocess calls
pixi_bin = r"C:\pixi_ws\.pixi\envs\default\Library\bin"
os.environ["PATH"] = pixi_bin + os.pathsep + os.environ["PATH"]

from rosa_mcp_server.server import ros_status, actuator_list, ros_topic_list

print("--- ROS STATUS ---")
print(ros_status())
print("\n--- SIMULATED ACTUATORS ---")
print(actuator_list())
print("\n--- ROS TOPICS ---")
print(ros_topic_list())
