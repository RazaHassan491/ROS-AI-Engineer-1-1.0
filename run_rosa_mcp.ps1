# ROSA MCP Server Startup Script for Windows
# This script prepares the ROS2 environment and starts the MCP server.

$ErrorActionPreference = "Stop"

# Paths
$PIXI_ENV = "C:\pixi_ws\.pixi\envs\default"
$PYTHON_EXE = "$PIXI_ENV\python.exe"
$PROJECT_ROOT = Get-Item -Path ".."
$SRC_DIR = Join-Path $PSScriptRoot "src"



# Set Environment Variables
$env:ROS_VERSION = "2"
$env:ROS_DISTRO = "humble"
$env:ROS_DOMAIN_ID = "0"
$env:AMENT_PREFIX_PATH = "$PIXI_ENV\Library"
$env:PYTHONPATH = "$PIXI_ENV\Library\lib\site-packages;$PIXI_ENV\Lib\site-packages;$SRC_DIR"

# Ensure PIXI env is in PATH for DLLs and ROS tools
$env:PATH = "$PIXI_ENV\Library\bin;$PIXI_ENV\Scripts;$PIXI_ENV;$env:PATH"

# Pre-start ROS 2 Daemon to reduce 'Analyzing' delay
Write-Host "Pre-warming ROS 2 environment..."
Start-Process -FilePath "$PIXI_ENV\Library\bin\ros2.exe" -ArgumentList "daemon start" -NoNewWindow -ErrorAction SilentlyContinue

# Run the server
Write-Host "Starting ROSA MCP Server..."
& "$PYTHON_EXE" -m rosa_mcp_server
