"""FastMCP server exposing UR3Controller as MCP tools.

Run with:
    uv run python robot/mcp_server.py
or
    fastmcp run robot/mcp_server.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from fastmcp import FastMCP
from ur3_controller import UR3Controller


mcp = FastMCP("UR3 Robot Controller")

# Singleton robot instance — connect/disconnect tools manage its lifecycle.
robot = UR3Controller()


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

mcp.add_tool(robot.connect)
mcp.add_tool(robot.disconnect)


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------

mcp.add_tool(robot.move_to_xyz_j)


# ---------------------------------------------------------------------------
# Gripper
# ---------------------------------------------------------------------------

mcp.add_tool(robot.gripper_activate)
mcp.add_tool(robot.grab_object)
mcp.add_tool(robot.release_object)



if __name__ == "__main__":
    mcp.run()
