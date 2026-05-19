import asyncio
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from fastmcp import FastMCP
from ur3_controller import UR3Controller


async def main():
    # Start the HTTP transport so agents can call the registered tool.
    await mcp.run_async(transport="http", port=8000)

if __name__ == "__main__":

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




    asyncio.run(main())
