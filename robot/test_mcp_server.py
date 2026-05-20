import asyncio
from fastmcp import Client

async def main():
    async with Client("http://127.0.0.1:8001/mcp") as client:
        # Test connectivity
        await client.ping()
        # Call a tool
        
        result = await client.call_tool("move_robot", {"coordinates": [0,0,0,0,0,0]})
        print(f"Result: {result}")

if __name__ == "__main__":
    asyncio.run(main())