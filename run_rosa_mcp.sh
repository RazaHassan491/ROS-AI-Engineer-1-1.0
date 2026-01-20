#!/bin/bash
# Wrapper script to run ROSA MCP server with ROS2 environment

# Deactivate conda if active
export CONDA_PREFIX=""
export CONDA_DEFAULT_ENV=""

# Clear Python paths that might interfere
unset PYTHONPATH

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Add rosa_mcp_server to PYTHONPATH
export PYTHONPATH="/home/sirapop/Documents/ROS-AI-Engineer-1.0/ROS-AI-Engineer-1.0-main/src:$PYTHONPATH"

# Install mcp if not present (first run only)
/usr/bin/python3 -c "import mcp" 2>/dev/null || /usr/bin/python3 -m pip install --user mcp

# Run the MCP server with system Python 3.10
cd /home/sirapop/Documents/ROS-AI-Engineer-1.0/ROS-AI-Engineer-1.0-main
exec /usr/bin/python3 -m rosa_mcp_server "$@"
