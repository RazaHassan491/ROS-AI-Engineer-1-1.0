"""
ROSA MCP Server - Main server implementation.

This module creates an MCP server using FastMCP that exposes ROSA's ROS tools
for use with Gemini CLI and other MCP-compatible clients.
"""

import os
import sys
import logging
from typing import Optional
import io
from contextlib import redirect_stdout, redirect_stderr
from mcp.server.fastmcp import FastMCP


# NUCLEAR WINDOWS FIX: Hardcode environment before ANYTHING else loads
PIXI_ENV = r"C:\pixi_ws\.pixi\envs\default"
os.environ["AMENT_PREFIX_PATH"] = os.path.join(PIXI_ENV, "Library")
os.environ["ROS_VERSION"] = "2"
os.environ["ROS_DOMAIN_ID"] = "0"
os.environ["ROS_DISTRO"] = "humble"

# Force PIXI bin to the front of PATH for DLL resolution
pixi_bin = os.path.join(PIXI_ENV, "Library", "bin")
if pixi_bin not in os.environ.get("PATH", ""):
    os.environ["PATH"] = f"{pixi_bin};{os.environ.get('PATH', '')}"

# Ensure PYTHONPATH includes ROS 2 packages
ros_python_path = os.path.join(PIXI_ENV, "Library", "lib", "site-packages")
ros_python_path_2 = os.path.join(PIXI_ENV, "Lib", "site-packages")
if ros_python_path not in os.environ.get("PYTHONPATH", ""):
    os.environ["PYTHONPATH"] = f"{ros_python_path};{ros_python_path_2};{os.environ.get('PYTHONPATH', '')}"

# Verify PYTHONPATH includes src
src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
os.environ["PYTHONPATH"] = f"{src_dir};{os.environ.get('PYTHONPATH', '')}"


# Configure logging to stderr and a local file for deep debugging on Windows
DEBUG_LOG = r"C:\Users\Raza Hassan\rosa_mcp_debug.log"
def log_debug(msg):
    try:
        with open(DEBUG_LOG, "a") as f:
            f.write(f"{msg}\n")
    except Exception:
        pass 


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    stream=sys.stderr,
)
logger = logging.getLogger("rosa-mcp-server")
# log_debug("--- Server Script Loaded ---") # Moved inside main or tools to be safe



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

import concurrent.futures

# Thread pool for running potentially hanging ROS2 calls
_executor = concurrent.futures.ThreadPoolExecutor(max_workers=5)

def run_with_timeout(func, timeout=5, *args, **kwargs):
    """Run a function in a thread with a strict timeout."""
    future = _executor.submit(func, *args, **kwargs)
    try:
        return future.result(timeout=timeout)
    except concurrent.futures.TimeoutError:
        log_debug(f"TIMEOUT: {func.__name__} hung for > {timeout}s")
        return None
    except Exception as e:
        log_debug(f"ERROR in {func.__name__}: {e}")
        return None

# Combined ROS2 availability check with lazy init
def _check_ros_available() -> tuple[bool, str]:
    """Check if ROS 2 is available and initialized."""
    try:
        import rclpy
        if not rclpy.ok():
            # Initializing rclpy can hang, so we use a small timeout here too
            res = run_with_timeout(rclpy.init, timeout=3)
            if res is None and not rclpy.ok():
                return False, "ROS2 rclpy.init() timed out"
        return True, "ROS2 (rclpy) is available"
    except ImportError:
        return False, "ROS2 Python packages (rclpy) not installed"
    except Exception as e:
        return False, f"ROS2 init error: {e}"

# Persistent ROS2 Node for introspection
_global_node = None

def get_ros_node():
    """Get or create the global ROS2 node."""
    global _global_node
    if _global_node is None:
        try:
            available, msg = _check_ros_available()
            if not available:
                log_debug(f"ROS2 check failed: {msg}")
                return None
            
            import rclpy
            _global_node = run_with_timeout(rclpy.create_node, timeout=3, node_name="rosa_mcp_introspection")
            if _global_node is None:
                log_debug("Failed to create ROS2 node within timeout")
        except Exception as e:
            log_debug(f"Failed to create ROS2 node: {e}")
    return _global_node




@mcp.tool()
def ros_topic_list(pattern: Optional[str] = None) -> str:
    """
    List all active ROS 2 topics.
    
    Args:
        pattern: Optional regex pattern to filter topics (e.g., "/camera.*")
    """
    log_debug(f"Entering ros_topic_list(pattern={pattern})")
    try:
        node = get_ros_node()
        if not node:
            return "Error: ROS 2 node could not be initialized."

        # Use timeout guard for near-instant return even if DDS hangs
        topics_and_types = run_with_timeout(node.get_topic_names_and_types, timeout=5)
        if topics_and_types is None:
            return "Error: Topic discovery timed out (5s). ROS 2 daemon might be unresponsive."

        
        import re
        regex = re.compile(pattern) if pattern else None
        
        output = []
        for name, types in topics_and_types:
            if not regex or regex.search(name):
                output.append(f"{name} [{', '.join(types)}]")
        
        if not output:
            return "No topics found matching the pattern." if pattern else "No active ROS 2 topics found."
            
        return "\n".join(output)
    except Exception as e:
        log_debug(f"ros_topic_list failure: {e}")
        return f"Error listing topics: {e}"

@mcp.tool()
def ros_node_list(pattern: Optional[str] = None) -> str:
    """
    List all active ROS 2 nodes.
    
    Args:
        pattern: Optional regex pattern to filter nodes.
    """
    log_debug(f"Entering ros_node_list(pattern={pattern})")
    try:
        node = get_ros_node()
        if not node:
            return "Error: ROS 2 node could not be initialized."

        # Use timeout guard
        nodes = run_with_timeout(node.get_node_names, timeout=5)
        if nodes is None:
            return "Error: Node discovery timed out (5s)."

        
        import re
        regex = re.compile(pattern) if pattern else None
        
        output = []
        for name in nodes:
            if not regex or regex.search(name):
                output.append(name)
        
        if not output:
            return "No nodes found matching the pattern." if pattern else "No active ROS 2 nodes found."
            
        return "\n".join(output)
    except Exception as e:
        log_debug(f"ros_node_list failure: {e}")
        return f"Error listing nodes: {e}"



@mcp.tool()
def ros_topic_info(topics: str) -> str:
    """Get detailed information about specific ROS topics."""
    log_debug(f"Entering ros_topic_info(topics={topics})")
    try:
        node = get_ros_node()
        if not node: return "Error: ROS 2 node not initialized."
        
        output = []
        for topic in [t.strip() for t in topics.split(",")]:
            pubs = node.get_publishers_info_by_topic(topic)
            subs = node.get_subscriptions_info_by_topic(topic)
            output.append(f"Topic: {topic}")
            output.append(f"  Publishers: {len(pubs)}")
            for p in pubs: output.append(f"    - {p.node_name} ({p.topic_type})")
            output.append(f"  Subscribers: {len(subs)}")
            for s in subs: output.append(f"    - {s.node_name} ({s.topic_type})")
        return "\n".join(output)
    except Exception as e:
        return f"Error getting topic info: {e}"

@mcp.tool()
def ros_node_info(nodes: str) -> str:
    """Get detailed information about specific ROS nodes."""
    log_debug(f"Entering ros_node_info(nodes={nodes})")
    try:
        node = get_ros_node()
        if not node: return "Error: ROS 2 node not initialized."
        
        output = []
        for node_name in [n.strip() for n in nodes.split(",")]:
            # Simple version for speed
            output.append(f"Node: {node_name}")
            try:
                pubs = node.get_publisher_names_and_types_by_node(node_name, "/")
                subs = node.get_subscriber_names_and_types_by_node(node_name, "/")
                output.append("  Publications:")
                for n, t in pubs: output.append(f"    - {n} [{', '.join(t)}]")
                output.append("  Subscriptions:")
                for n, t in subs: output.append(f"    - {n} [{', '.join(t)}]")
            except Exception as e:
                output.append(f"  Info error: {e}")
        return "\n".join(output)
    except Exception as e:
        return f"Error getting node info: {e}"



# =============================================================================
# Health & Topic Tools
# =============================================================================

@mcp.tool()
def ping() -> str:
    """Simple health check tool."""
    log_debug("Tool 'ping' called")
    return "pong"



@mcp.tool()
def ros_service_list(pattern: Optional[str] = None) -> str:
    """List all active ROS 2 services."""
    log_debug(f"Entering ros_service_list(pattern={pattern})")
    try:
        node = get_ros_node()
        if not node: return "Error: ROS 2 node not initialized."
        
        # Use timeout guard
        services = run_with_timeout(node.get_service_names_and_types, timeout=5)
        if services is None:
            return "Error: Service discovery timed out (5s)."

        import re
        regex = re.compile(pattern) if pattern else None
        
        output = []
        for name, types in services:
            if not regex or regex.search(name):
                output.append(f"{name} [{', '.join(types)}]")
        return "\n".join(output) if output else "No services found."
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
def ros_param_list() -> str:
    """List all active ROS 2 parameters."""
    log_debug("Entering ros_param_list()")
    try:
        # Param discovery is slower, we focus on listing currently active nodes' params
        node = get_ros_node()
        if not node: return "Error: ROS 2 node not initialized."
        
        output = ["Global parameters (via discovery):"]
        # Note: True param listing in ROS2 usually requires individual node queries
        # We'll return the nodes first to help the user know where to look
        nodes = node.get_node_names()
        output.append(f"Active Nodes: {', '.join(nodes)}")
        output.append("\nUse 'ros_param_get' with /node_name/parameter_name")
        return "\n".join(output)
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
    
    # DYNAMIC SEARCH: Find script and pixi.toml by walking up from THIS file
    # This file is in src/rosa_mcp_server/server.py
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    
    script_name = "capture_images.py"
    pixi_toml_name = "pixi.toml"
    pixi_exe = r"C:\Users\Raza Hassan\.pixi\bin\pixi.exe"
    
    script_path = None
    pixi_toml_path = None
    project_root = None
    
    # Walk up to 4 levels to find the project root
    check_dir = current_file_dir
    for _ in range(5):
        potential_script = os.path.join(check_dir, script_name)
        potential_pixi = os.path.join(check_dir, pixi_toml_name)
        
        if os.path.exists(potential_script) and script_path is None:
            script_path = potential_script
            project_root = check_dir
            
        if os.path.exists(potential_pixi) and pixi_toml_path is None:
            pixi_toml_path = potential_pixi
            
        if script_path and pixi_toml_path:
            break
            
        parent = os.path.dirname(check_dir)
        if parent == check_dir: break # Root reached
        check_dir = parent

    # Fallbacks if dynamic search fails (using last known good paths)
    if not script_path:
        # User confirmed location
        script_path = r"C:\Users\Raza Hassan\Downloads\ROS-AI-Engineer-1.0-main\ROS-AI-Engineer-1.0-main\capture_images.py"
        project_root = os.path.dirname(script_path)
    if not pixi_toml_path:
        pixi_toml_path = r"C:\pixi_ws\pixi.toml"

    if not os.path.exists(script_path):
        return f"Error: Could not find {script_name}. Searched up to 4 levels from {current_file_dir}. Last path tried: {script_path}"

    # Ensure output directory exists (script expects 'camera_images')
    try:
        os.makedirs(os.path.join(project_root, "camera_images"), exist_ok=True)
    except Exception:
        pass
    
    try:
        # ENVIRONMENT IS KEY: Use pixi run to replicate the successful manual capture environment
        cmd = [
            pixi_exe, 
            "run", 
            "--manifest-path", pixi_toml_path, 
            "python", script_path
        ]
        
        env = os.environ.copy()
        env["AMENT_PREFIX_PATH"] = os.path.join(PIXI_ENV, "Library")
        env["ROS_VERSION"] = "2"
        env["ROS_DOMAIN_ID"] = "0"
        
        log_debug(f"Running snapshot cmd: {' '.join(cmd)}")
        
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=45,
            env=env,
            cwd=project_root
        )

        if result.returncode == 0:
            return f"Snapshot captured successfully!\nOutput:\n{result.stdout}\n\nImages saved in 'camera_images/' folder."
        else:
            return f"Failed to capture snapshot.\nError:\n{result.stderr}\nOutput:\n{result.stdout}\nCommand run: {' '.join(cmd)}\nIn directory: {project_root}"
            
    except subprocess.TimeoutExpired:
        return f"Error: Timed out (45s) waiting for {script_name}. Check if camera topics are active."
    except Exception as e:
        return f"Error running capture script via pixi: {e}"


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
    log_debug("\n--- NEW SESSION STARTED ---")
    log_debug(f"Python: {sys.version}")
    log_debug(f"CWD: {os.getcwd()}")
    
    # Redirect noisy library startup messages to stderr
    with redirect_stdout(sys.stderr):
        logger.info(f"Starting ROSA MCP Server (Lean Mode)")
        # Defer check_ros until a tool is actually used to save time
        log_debug("--- Environment verified, ready to enter main loop ---")

    # CRITICAL: mcp.run() MUST be outside redirect_stdout to use the correct stdout pipe for MCP protocol!
    log_debug("--- Entering mcp.run() on STDOUT ---")
    mcp.run()







if __name__ == "__main__":
    main()
