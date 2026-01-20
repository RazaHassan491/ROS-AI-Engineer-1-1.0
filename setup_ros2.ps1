# ROS 2 Environment Setup for Windows (Pixi)
# Usage: . .\setup_ros2.ps1

$PIXI_ENV = "C:\pixi_ws\.pixi\envs\default"
$PROJECT_ROOT = $PSScriptRoot

Write-Host "Setting up ROS 2 environment (Humble) from Pixi..." -ForegroundColor Cyan

$env:ROS_VERSION = "2"
$env:ROS_DISTRO = "humble"
$env:ROS_DOMAIN_ID = "0"
$env:AMENT_PREFIX_PATH = "$PIXI_ENV\Library"
$env:PYTHONPATH = "$PIXI_ENV\Library\lib\site-packages;$PIXI_ENV\Lib\site-packages;$PROJECT_ROOT\src;$env:PYTHONPATH"

# PATH is critical for DLL resolution
$env:PATH = "$PIXI_ENV\Library\bin;$PIXI_ENV\Scripts;$PIXI_ENV;$env:PATH"

Write-Host "ROS 2 Environment Staged." -ForegroundColor Green
Write-Host "Prefix: $env:AMENT_PREFIX_PATH"
Write-Host "Python: $PIXI_ENV\python.exe"
