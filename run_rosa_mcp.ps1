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
$env:PYTHONPATH = "$SRC_DIR;$env:PYTHONPATH"
$env:AMENT_PREFIX_PATH = "$PIXI_ENV"
$env:ROS_DOMAIN_ID = "0"


# Ensure PIXI env is in PATH for DLLs
$env:PATH = "$PIXI_ENV\Library\bin;$PIXI_ENV\Scripts;$PIXI_ENV;$env:PATH"

# Run the server
& "$PYTHON_EXE" -m rosa_mcp_server
