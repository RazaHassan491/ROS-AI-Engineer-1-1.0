"""
ROSA MCP Server - Bridge between ROSA and Gemini CLI via MCP protocol.

This package provides an MCP server that exposes ROSA's ROS tools,
enabling natural language interaction with ROS systems through Gemini CLI.
"""

from rosa_mcp_server.server import mcp, main

__version__ = "0.1.0"
__all__ = ["mcp", "main"]
