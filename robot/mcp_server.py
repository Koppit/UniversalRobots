import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from fastmcp import FastMCP
from ur3_controller import UR3Controller


if __name__ == "__main__":

    mcp = FastMCP("UR3 Robot Controller")

    # Singleton robot instance — connect/disconnect tools manage its lifecycle.
    robot = UR3Controller(ip="192.168.0.25")

    try:
        robot.connect()

        # ---------------------------------------------------------------------------
        # Motion
        # ---------------------------------------------------------------------------

        mcp.add_tool(robot.move_robot)

        # ---------------------------------------------------------------------------
        # Gripper
        # ---------------------------------------------------------------------------

        mcp.add_tool(robot.gripper_activate)
        mcp.add_tool(robot.grab_object)
        mcp.add_tool(robot.release_object)

        # ---------------------------------------------------------------------------
        # Start MCP Server
        # ---------------------------------------------------------------------------

        mcp.run(transport="http", port=8001)

    finally:

        robot.disconnect()