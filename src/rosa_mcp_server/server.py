"""
ROSA MCP Server - Main server implementation.

This module creates an MCP server using FastMCP that exposes ROSA's ROS tools
for use with Gemini CLI and other MCP-compatible clients.
"""

import os
import sys
import logging
from typing import Optional

from mcp.server.fastmcp import FastMCP
import io
from contextlib import redirect_stdout, redirect_stderr


# Configure logging to stderr (stdout is reserved for MCP protocol)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    stream=sys.stderr,
)
logger = logging.getLogger("rosa-mcp-server")

# Detect ROS version from environment
ROS_VERSION = os.environ.get("ROS_VERSION", "2")

# Force ROS 2 environment for this project context
os.environ["ROS_VERSION"] = "2"
ROS_VERSION = "2"

# Create the MCP server with very explicit instructions to prevent AI hallucination
mcp = FastMCP(
    name="ROSA",
    instructions="Robot Operating System Agent (ROSA) for ROS 2 Humble on Windows. "
    "You are equipped with modern ROS 2 tools. If you encounter an error, do NOT claim you are ROS 1 based. "
    "You have full access to ROS 2 topics, nodes, and camera snapshots via the provided tools.",
)



def _check_ros_available() -> tuple[bool, str]:
    """Check if ROS is available in the current environment."""
    if ROS_VERSION == "1":
        try:
            import rospy
            import rosgraph

            master = rosgraph.Master("/rosa_mcp_check")
            master.getSystemState()
            return True, "ROS1 Master is available"
        except ImportError:
            return False, "ROS1 Python packages (rospy) not installed"
        except Exception as e:
            return False, f"ROS1 Master not reachable: {e}"
    else:
        try:
            import rclpy

            return True, "ROS2 (rclpy) is available"
        except ImportError:
            return False, "ROS2 Python packages (rclpy) not installed"


# =============================================================================
# Topic Tools
# =============================================================================


@mcp.tool()
def ros_topic_list(pattern: Optional[str] = None) -> str:
    """
    List available topics in the ROS 2 Humble environment.


    Args:
        pattern: Optional regex pattern to filter topics (e.g., "/camera.*")

    Returns:
        A list of ROS topics matching the pattern, or all topics if no pattern.
    """
    try:
        import subprocess
        import os
        
        # Prepare a clean environment for the subprocess
        import os
        env = os.environ.copy()
        
        # Force default domain and search paths
        env["ROS_DOMAIN_ID"] = "0"
        env["AMENT_PREFIX_PATH"] = r"C:\pixi_ws\.pixi\envs\default"
        env["ROS_VERSION"] = "2"
        
        # Ensure PIXI bin is in PATH for internal DLLs
        pixi_bin = r"C:\pixi_ws\.pixi\envs\default\Library\bin"
        if pixi_bin not in env.get("PATH", ""):
            env["PATH"] = f"{pixi_bin};{env.get('PATH', '')}"


        # Absolute path to ros2.exe in the pixi env
        ros2_exe = r"C:\pixi_ws\.pixi\envs\default\Library\bin\ros2.exe"

        # Execute the command with absolute path and 15s timeout
        result = subprocess.run(
            [ros2_exe, "topic", "list"], 
            capture_output=True, 
            text=True, 
            timeout=15, 
            env=env
        )


        
        if result.returncode == 0:
            output = result.stdout.strip()
            if not output:
                return "No topics found. ROS2 may be starting up or no nodes are active."
            
            if pattern:
                import re
                lines = [l for l in output.splitlines() if re.search(pattern, l)]
                return "\n".join(lines) if lines else f"No topics matching pattern: {pattern}"
            
            return output
        else:
            return f"Error: ros2 topic list failed (code {result.returncode})\nStderr: {result.stderr}"
            
    except subprocess.TimeoutExpired:
        return "Error: Topic discovery timed out after 15 seconds. This usually happens if the ROS2 daemon is stuck or DDS multicast is blocked."
    except Exception as e:
        return f"Unexpected error in topic list: {str(e)}"



@mcp.tool()
def ros_topic_info(topics: str) -> str:
    """
    Get detailed information about specific ROS topics.

    Args:
        topics: Comma-separated list of topic names (e.g., "/cmd_vel,/odom")

    Returns:
        Details about each topic including type, publishers, and subscribers.
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    topic_list = [t.strip() for t in topics.split(",")]

    try:
        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rostopic_info

            result = rostopic_info.invoke({"topics": topic_list})
            return str(result)
        else:
            from rosa.tools.ros2 import ros2_topic_info

            result = ros2_topic_info.invoke({"topics": topic_list})
            return str(result)
    except Exception as e:
        return f"Error getting topic info: {e}"


@mcp.tool()
def ros_topic_echo(topic: str, count: int = 1, timeout: float = 5.0) -> str:
    """
    Echo messages from a ROS topic.

    Args:
        topic: The topic name to echo (e.g., "/cmd_vel")
        count: Number of messages to capture (1-10, default: 1)
        timeout: Timeout in seconds to wait for messages (default: 5.0)

    Returns:
        The captured messages from the topic.
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    # Clamp count to reasonable range
    count = max(1, min(10, count))

    try:
        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rostopic_echo

            result = rostopic_echo.invoke({
                "topic": topic, "count": count, "return_echoes": True, "timeout": timeout
            })
            return str(result)
        else:
            from rosa.tools.ros2 import ros2_topic_echo

            result = ros2_topic_echo.invoke({
                "topic": topic, "count": count, "return_echoes": True, "timeout": timeout
            })
            return str(result)
    except Exception as e:
        return f"Error echoing topic: {e}"


# =============================================================================
# Node Tools
# =============================================================================


@mcp.tool()
def ros_node_list(pattern: Optional[str] = None) -> str:
    """
    List running ROS nodes.

    Args:
        pattern: Optional regex pattern to filter nodes (e.g., "/robot.*")

    Returns:
        A list of running ROS nodes matching the pattern.
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    try:
        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rosnode_list

            result = rosnode_list.invoke({"pattern": pattern})
            return str(result)
        else:
            from rosa.tools.ros2 import ros2_node_list

            result = ros2_node_list.invoke({"pattern": pattern, "blacklist": None})
            return str(result)
    except Exception as e:
        return f"Error listing nodes: {e}"


@mcp.tool()
def ros_node_info(nodes: str) -> str:
    """
    Get detailed information about specific ROS nodes.

    Args:
        nodes: Comma-separated list of node names (e.g., "/turtlesim,/teleop")

    Returns:
        Details about each node including publications, subscriptions, and services.
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    node_list = [n.strip() for n in nodes.split(",")]

    try:
        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rosnode_info

            result = rosnode_info.invoke({"nodes": node_list})
            return str(result)
        else:
            from rosa.tools.ros2 import ros2_node_info

            result = ros2_node_info.invoke({"nodes": node_list})
            return str(result)
    except Exception as e:
        return f"Error getting node info: {e}"


# =============================================================================
# Service Tools
# =============================================================================


@mcp.tool()
def ros_service_list(pattern: Optional[str] = None) -> str:
    """
    List available ROS services.

    Args:
        pattern: Optional regex pattern to filter services (e.g., "/spawn.*")

    Returns:
        A list of available ROS services.
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    try:
        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rosservice_list

            result = rosservice_list.invoke({"regex_pattern": pattern})
            return str(result)
        else:
            from rosa.tools.ros2 import ros2_service_list

            result = ros2_service_list.invoke({"pattern": pattern, "blacklist": None})
            return str(result)
    except Exception as e:
        return f"Error listing services: {e}"


@mcp.tool()
def ros_service_call(service: str, args: Optional[str] = None) -> str:
    """
    Call a ROS service.

    Args:
        service: The service name (e.g., "/reset")
        args: Optional JSON string of arguments for the service

    Returns:
        The service response.

    Example:
        ros_service_call("/spawn", '{"x": 5.0, "y": 5.0, "name": "turtle2"}')
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    try:
        import json

        parsed_args = json.loads(args) if args else None

        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rosservice_call

            result = rosservice_call.invoke({
                "service": service, "args": list(parsed_args.values()) if parsed_args else []
            })
            return str(result)
        else:
            from rosa.tools.ros2 import ros2_service_call

            # ROS2 requires service type - this is a simplified version
            result = ros2_service_call.invoke({
                "service_name": service, "srv_type": "", "request": args or ""
            })
            return str(result)
    except json.JSONDecodeError as e:
        return f"Invalid JSON args: {e}"
    except Exception as e:
        return f"Error calling service: {e}"


# =============================================================================
# Parameter Tools
# =============================================================================


@mcp.tool()
def ros_param_list(namespace: str = "/") -> str:
    """
    List ROS parameters.

    Args:
        namespace: ROS namespace to list parameters from (default: "/")

    Returns:
        A list of ROS parameters in the given namespace.
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    try:
        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rosparam_list

            result = rosparam_list.invoke({"namespace": namespace})
            return str(result)
        else:
            from rosa.tools.ros2 import ros2_param_list

            result = ros2_param_list.invoke({})
            return str(result)
    except Exception as e:
        return f"Error listing parameters: {e}"


@mcp.tool()
def ros_param_get(params: str) -> str:
    """
    Get the value of ROS parameters.

    Args:
        params: Comma-separated list of parameter names (e.g., "/robot/speed,/robot/name")

    Returns:
        The values of the requested parameters.
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    param_list = [p.strip() for p in params.split(",")]

    try:
        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rosparam_get

            result = rosparam_get.invoke({"params": param_list})
            return str(result)
        else:
            # ROS2 param_get requires node name, simplified for now
            results = []
            for param in param_list:
                parts = param.split("/")
                if len(parts) >= 3:
                    node_name = "/" + parts[1]
                    param_name = "/".join(parts[2:])
                    from rosa.tools.ros2 import ros2_param_get

                    result = ros2_param_get.invoke({"node_name": node_name, "param_name": param_name})
                    results.append(f"{param}: {result}")
                else:
                    results.append(f"{param}: Invalid format (use /node/param)")
            return "\n".join(results)
    except Exception as e:
        return f"Error getting parameters: {e}"


@mcp.tool()
def ros_param_set(param: str, value: str) -> str:
    """
    Set a ROS parameter value.

    Args:
        param: The parameter name (e.g., "/robot/speed")
        value: The value to set (will be auto-converted to appropriate type)

    Returns:
        Confirmation of the parameter change.
    """
    available, msg = _check_ros_available()
    if not available:
        return f"ROS not available: {msg}"

    try:
        if ROS_VERSION == "1":
            from rosa.tools.ros1 import rosparam_set

            result = rosparam_set.invoke({"param": param, "value": value, "is_rosa_param": False})
            return str(result) if result else f"Parameter {param} set to {value}"
        else:
            parts = param.split("/")
            if len(parts) >= 3:
                node_name = "/" + parts[1]
                param_name = "/".join(parts[2:])
                from rosa.tools.ros2 import ros2_param_set

                result = ros2_param_set.invoke({
                    "node_name": node_name, "param_name": param_name, "param_value": value
                })
                return str(result)
            else:
                return f"Invalid param format: {param} (use /node/param)"
    except Exception as e:
        return f"Error setting parameter: {e}"


# =============================================================================
# Utility Tools
# =============================================================================


@mcp.tool()
def ros_status() -> str:
    """
    Check ROS environment status and connectivity.

    Returns:
        Information about ROS availability, version, and environment.
    """
    available, msg = _check_ros_available()

    status_lines = [
        f"ROS Version: {ROS_VERSION}",
        f"Available: {available}",
        f"Status: {msg}",
        "",
        "Environment Variables:",
        f"  ROS_VERSION: {os.environ.get('ROS_VERSION', 'not set')}",
    ]

    if ROS_VERSION == "1":
        status_lines.append(
            f"  ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI', 'not set')}"
        )
    else:
        status_lines.append(
            f"  ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'not set')}"
        )

    return "\n".join(status_lines)


@mcp.tool()
def ros_snapshot() -> str:
    """
    Capture a ROS 2 camera snapshot (Color and Depth) from /camera/color/image_raw.
    Uses rclpy to subscribe to the topics. Ensure the camera driver is running.

    
    Returns:
        Status message with paths to saved images.
    """
    import subprocess
    
    # Locate capture_images.py script
    # Check current dir and parent directories
    script_name = "capture_images.py"
    possible_paths = [
        os.path.join(os.getcwd(), script_name),
        os.path.join(os.getcwd(), "..", script_name),
        os.path.join(os.getcwd(), "..", "..", script_name),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", script_name))
    ]
    
    script_path = None
    for p in possible_paths:
        if os.path.exists(p):
            script_path = p
            break
            
    if not script_path:
        return f"Error: Could not find {script_name}. Searched in: {possible_paths}"
        
    # Ensure output directory exists (script expects 'camera_images')
    # We do this relative to the script location
    try:
        os.makedirs(os.path.join(os.path.dirname(script_path), "camera_images"), exist_ok=True)
    except Exception:
        pass # Ignore permission errors, script might handle it or fail later
    
    try:
        # Run the script using the current python interpreter
        # Use a timeout to prevent infinite hanging
        result = subprocess.run(
            [sys.executable, script_path],
            capture_output=True,
            text=True,
            timeout=30,
            env=os.environ.copy()
        )

        
        if result.returncode == 0:
            return f"Snapshot captured successfully!\nOutput:\n{result.stdout}\n\nImages saved in 'camera_images/' folder."
        else:
            return f"Failed to capture snapshot.\nError:\n{result.stderr}\nOutput:\n{result.stdout}"
            
    except subprocess.TimeoutExpired:
        return "Error: Timed out waiting for image capture. Check if camera is connected and publishing."
    except Exception as e:
        return f"Error running capture script: {e}"


# =============================================================================
# Actuator Simulation Tools (No ROS Required)
# =============================================================================

from rosa_mcp_server.actuators import actuator_manager


@mcp.tool()
def actuator_list() -> str:
    """
    List all simulated actuators and their status.

    Returns:
        A list of actuators with their type, position, and status.
    """
    actuators = actuator_manager.list_actuators()
    
    # Format as a nice table
    lines = ["Simulated Actuators:", ""]
    for a in actuators:
        lines.append(f"- {a['name']} ({a['type']}):")
        lines.append(f"  Position: {a['position']:.1f} (Range: {a['min']} to {a['max']})")
        lines.append(f"  Moving: {a['is_moving']}")
    
    return "\n".join(lines)


@mcp.tool()
def actuator_move(name: str, position: float) -> str:
    """
    Move a simulated actuator to a target position.

    Args:
        name: Name of the actuator (e.g., "arm_base", "gripper")
        position: Target position (degrees for servos, % for others)

    Returns:
        Status message about the movement.
    """
    return actuator_manager.move_actuator(name, position)


@mcp.tool()
def actuator_status(name: str) -> str:
    """
    Get detailed status of a specific actuator.

    Args:
        name: Name of the actuator.

    Returns:
        Current status including position and limits.
    """
    status = actuator_manager.get_status(name)
    if "error" in status:
        return status["error"]
    
    return (
        f"Actuator: {status['name']}\n"
        f"Type: {status['type']}\n"
        f"Position: {status['position']:.1f}\n"
        f"Range: {status['min']} - {status['max']}\n"
        f"Status: {'Moving' if status['is_moving'] else 'Idle'}"
    )


# =============================================================================
# Main Entry Point
# =============================================================================


def main():
    """Run the ROSA MCP server."""
    logger.info(f"Starting ROSA MCP Server (ROS{ROS_VERSION})")
    available, msg = _check_ros_available()
    if available:
        logger.info(f"ROS Status: {msg}")
    else:
        logger.warning(f"ROS Status: {msg}")
        logger.warning("Tools will return error messages until ROS is available")

    # Run with stdio transport (default for MCP)
    mcp.run()


if __name__ == "__main__":
    main()
