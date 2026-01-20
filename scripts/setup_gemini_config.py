import os
import json
import sys
import shutil
from pathlib import Path

def setup_config():
    """Add ROSA MCP server to Gemini CLI configuration."""
    print("🤖 ROSA MCP Server Setup Helper")
    print("===============================")

    # Find Gemini CLI settings
    user_home = Path.home()
    gemini_dir = user_home / ".gemini"
    settings_path = gemini_dir / "settings.json"

    if not settings_path.exists():
        print(f"❌ Gemini CLI settings not found at {settings_path}")
        print("   Please run 'gemini' once to initialize it.")
        return

    print(f"📂 Found settings at: {settings_path}")

    # Create backup
    backup_path = settings_path.with_suffix(".json.bak")
    shutil.copy2(settings_path, backup_path)
    print(f"💾 Created backup at: {backup_path}")

    # Read config
    try:
        with open(settings_path, "r") as f:
            config = json.load(f)
    except json.JSONDecodeError:
        print("❌ Error reading settings.json")
        return

    # Determine current path
    project_root = Path(__file__).parent.parent.resolve()
    print(f"📍 Project root: {project_root}")

    # Prepare MCP config
    mcp_config = {
        "command": "python",  # Use 'python' instead of 'py' for cross-platform compatibility
        "args": ["-m", "rosa_mcp_server"],
        "cwd": str(project_root),
        "timeout": 30000
    }

    # Update config
    if "mcpServers" not in config:
        config["mcpServers"] = {}
    
    config["mcpServers"]["rosa"] = mcp_config

    # Write back
    with open(settings_path, "w") as f:
        json.dump(config, f, indent=2)

    print("✅ Configuration updated successfully!")
    print("\nNext steps:")
    print("1. Install the package: pip install -e .")
    print("2. Restart Gemini CLI")
    print("3. Run: gemini mcp list")

if __name__ == "__main__":
    setup_config()
