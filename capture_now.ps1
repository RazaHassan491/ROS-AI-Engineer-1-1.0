# ROS 2 Capture Wrapper (Ver 3.0 - Total Reset)
# This script kills stale processes to fix "0 Subscribers" issue

$PIXI_ENV = "C:\pixi_ws\.pixi\envs\default"
$SCRIPT_DIR = $PSScriptRoot

Write-Host "--- ROSA EMERGENCY CAPTURE (Ver 3.0) ---"

# 1. KILL STALE DAEMONS & PUBLISHERS (Flushing discovery)
Write-Host "Resetting ROS 2 daemons..."
$env:PATH = "$PIXI_ENV\Library\bin;$env:PATH"
& "ros2.exe" daemon stop 2>$null
& "ros2.exe" daemon start 2>$null

# 2. Environment Variables
$env:ROS_VERSION = "2"
$env:ROS_DISTRO = "humble"
$env:ROS_DOMAIN_ID = "0"
$env:AMENT_PREFIX_PATH = "$PIXI_ENV\Library"
$env:PYTHONPATH = "$PIXI_ENV\Library\lib\site-packages;$PIXI_ENV\Lib\site-packages;$SCRIPT_DIR\src;$env:PYTHONPATH"

# 3. Path for DLLs
$env:PATH = "$PIXI_ENV\Library\bin;$PIXI_ENV\Scripts;$PIXI_ENV;$env:PATH"

# 4. Clear old life-sign log
Remove-Item "$SCRIPT_DIR\capture_life_sign.log" -ErrorAction SilentlyContinue

# 5. RUN CAPTURE
Write-Host "Capturing images..."
& "$PIXI_ENV\python.exe" "$SCRIPT_DIR\capture_images.py"

# 6. Show status
if (Test-Path "$SCRIPT_DIR\camera_images\color_image.png") {
    Write-Host "SUCCESS: Image found on disk."
} else {
    Write-Host "FAILED: Check capture_life_sign.log"
}
