import sys
import os

# Minimal MCP Protocol implementation (no FastMCP)
def main():
    # Write to a log to confirm instant start
    log_path = os.path.join(os.path.expanduser("~"), "mcp_minimal_speed.log")
    with open(log_path, "a") as f:
        f.write("Minimal Server Started Instantly!\n")
    
    # Just loop and do nothing, pretending to be a server
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        # Echo back a fake list of tools if asked (basic protocol)
        if '"method":"list_tools"' in line:
            sys.stdout.write('{"jsonrpc":"2.0","id":1,"result":{"tools":[]}}\n')
            sys.stdout.flush()

if __name__ == "__main__":
    main()
