import os
import subprocess
import sys

# Environment Setup
PIXI_ENV = r"C:\pixi_ws\.pixi\envs\default"
PYTHON_EXE = os.path.join(PIXI_ENV, "python.exe")
WORKING_DIR = r"c:\Users\Raza Hassan\Downloads\ROS-AI-Engineer-1.0-main\ROS-AI-Engineer-1.0-main\ROS-AI-Engineer-1.0-main"

env = os.environ.copy()
env["AMENT_PREFIX_PATH"] = PIXI_ENV
env["ROS_VERSION"] = "2"
env["ROS_DOMAIN_ID"] = "0"
pixi_bin = os.path.join(PIXI_ENV, "Library", "bin")
env["PATH"] = f"{pixi_bin};{env.get('PATH', '')}"

print(f"Launching camera publisher from {WORKING_DIR}...")
print(f"Using Python: {PYTHON_EXE}")

try:
    with open("camera_output.log", "w") as out, open("camera_error.log", "w") as err:
        proc = subprocess.Popen(
            [PYTHON_EXE, "realsense_publisher.py"],
            cwd=WORKING_DIR,
            env=env,
            stdout=out,
            stderr=err,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.DETACHED_PROCESS if os.name == 'nt' else 0
        )
    print(f"Started camera publisher with PID: {proc.pid}")
    print("Logs redirected to camera_output.log and camera_error.log")
except Exception as e:
    print(f"Failed to launch: {e}")
