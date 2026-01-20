# ROSA MCP Server

An MCP (Model Context Protocol) server that bridges [ROSA](https://github.com/nasa-jpl/rosa) (Robot Operating System Agent) with [Gemini CLI](https://github.com/google-gemini/gemini-cli), enabling natural language interaction with ROS systems.

## Features

- 🤖 **Natural Language ROS Control**: Query and control ROS systems using plain English
- 🔌 **MCP Protocol**: Standard integration with Gemini CLI and other MCP-compatible clients
- 🔄 **ROS1 & ROS2 Support**: Auto-detects ROS version from environment
- 🛡️ **Graceful Degradation**: Works without ROS for testing MCP integration

## Installation

```bash
# Clone or navigate to the rosa-mcp-server directory
cd rosa-mcp-server

# Install with pip
pip install -e .

# Or with ROS support
pip install -e ".[ros1]"  # For ROS1
pip install -e ".[ros2]"  # For ROS2
```

## Usage

### Run the MCP Server

```bash
# Using the installed script
rosa-mcp-server

# Or using Python module
python -m rosa_mcp_server
```

### Configure Gemini CLI

Add to `~/.gemini/settings.json`:

```json
{
  "mcpServers": {
    "rosa": {
      "command": "python",
      "args": ["-m", "rosa_mcp_server"],
      "cwd": "/path/to/rosa-mcp-server",
      "timeout": 30000
    }
  }
}
```

### Example Commands in Gemini CLI

```
> @rosa List all ROS topics
> @rosa Show me the nodes that are currently running
> @rosa What is the value of the /robot/speed parameter?
> @rosa Call the /reset service
```

## Available Tools

| Tool | Description |
|------|-------------|
| `ros_topic_list` | List available ROS topics |
| `ros_topic_info` | Get details about specific topics |
| `ros_topic_echo` | Echo messages from a topic |
| `ros_node_list` | List running ROS nodes |
| `ros_node_info` | Get details about specific nodes |
| `ros_service_list` | List available ROS services |
| `ros_service_call` | Call a ROS service |
| `ros_param_list` | List ROS parameters |
| `ros_param_get` | Get parameter values |
| `ros_param_set` | Set parameter values |

## Environment Variables

- `ROS_VERSION`: Set to `1` or `2` to specify ROS version (auto-detected if not set)
- `ROS_MASTER_URI`: ROS1 Master URI (default: `http://localhost:11311`)
- `ROS_DOMAIN_ID`: ROS2 Domain ID (default: `0`)

## License

Apache-2.0
