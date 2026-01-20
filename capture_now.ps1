# ROS 2 Capture Wrapper for Windows
# This script ensures a 100% perfect environment before running capture_images.py

$PIXI_ENV = "C:\pixi_ws\.pixi\envs\default"
$SCRIPT_DIR = $PSScriptRoot

Write-Host "--- INITIALIZING ROS 2 CAPTURE (Ver 2.5) ---"

# 1. Environment Variables
$env:ROS_VERSION = "2"
$env:ROS_DISTRO = "humble"
$env:ROS_DOMAIN_ID = "0"
$env:AMENT_PREFIX_PATH = "$PIXI_ENV\Library"
$env:PYTHONPATH = "$PIXI_ENV\Library\lib\site-packages;$PIXI_ENV\Lib\site-packages;$SCRIPT_DIR\src;$env:PYTHONPATH"

# 2. Path for DLLs
$env:PATH = "$PIXI_ENV\Library\bin;$PIXI_ENV\Scripts;$PIXI_ENV;$env:PATH"

# 3. Output directory
if (!(Test-Path "$SCRIPT_DIR\camera_images")) {
    New-Item -ItemType Directory -Path "$SCRIPT_DIR\camera_images" -Force
}

# 4. RUN CAPTURE
Write-Host "Running: python $SCRIPT_DIR\capture_images.py"
& "$PIXI_ENV\python.exe" "$SCRIPT_DIR\capture_images.py"
