# ROS-AI-Engineer Gemini Integration

This repository contains a ROS 2 based AI engineering environment optimized for Gemini CLI and Windows.

## 🚀 Quick Start

1. **Start the Camera Driver**:
   Ensure your RealSense camera is connected and run:
   ```powershell
   python Realsense_Publisher.py
   ```

2. **Run Gemini CLI**:
   In a fresh terminal, run:
   ```powershell
   gemini
   ```

3. **Take a Snapshot**:
   Ask Gemini: *"Use the camera to take a snapshot"*

## 🛠 Project Structure

- `src/rosa_mcp_server`: The core MCP server providing ROS 2 tools to Gemini.
- `capture_now.ps1`: Automated capture wrapper that resets ROS 2 daemons to fix discovery issues.
- `capture_images.py`: ROS 2 node that subscribes to camera topics and saves PNGs.
- `setup_ros2.ps1`: Manual environment setup script.

## 🔧 Troubleshooting

- **Hanging Snapshot**: If Gemini hangs while taking a picture, it's usually because the Windows ROS 2 system hasn't "discovered" the camera yet. Wait up to 2 minutes or try again.
- **Environment Error**: Always run `gemini` from a clean terminal where no other ROS 2 versions are sourced.

## 🔄 Updating
This project uses **ROSA MCP Snapshot Tool Ver 3.0**. To ensure you have the latest fix, restart your Gemini CLI terminal.
