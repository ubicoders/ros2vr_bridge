import asyncio
import websockets

from .ros2ws_utils import CMDS_DICT, send_cmd_msg
from .vr_msg_py.EMPT_empty_generated import EmptyMsg
from .vr_msg_py.R000_states_generated import StatesMsg

CONNECTIONS = set()


async def register(websocket, ros_publisher_cb):
    CONNECTIONS.add(websocket)
    try:
        async for message in websocket:
            if EmptyMsg.EmptyMsgBufferHasIdentifier(message, 0) is True:
                continue
            if message == b"ping" or message == "ping":
                await websocket.send(b"pong")
                continue
            if (StatesMsg.StatesMsgBufferHasIdentifier(message, 0) is True):
                ros_publisher_cb(message)

            # for cmd msgs
            await send_cmd_msg(websocket)
            # for service msgs
            others = CONNECTIONS - {websocket}
            websockets.broadcast(others, message)


    except Exception as e:
        pass
    finally:
        pass


async def manage_connections():
    while True:
        connections = list(CONNECTIONS)
        for websocket in connections:
            try:
                await websocket.wait_closed()
            finally:
                # print("removing connection")
                CONNECTIONS.remove(websocket)
                # print(f"size of connections: {len(CONNECTIONS)}")
        await asyncio.sleep(1)



async def run_ws_server(ros_publisher_cb, port=12740):
    
    # ros_publisher_cb = ros_publisher_cb
    async def register_with_cb(websocket):
        await register(websocket, ros_publisher_cb)

    # start server
    async with websockets.serve(register_with_cb, "localhost", port):
        print(f"\033[92mVirtual Robots Bridge is running @ port {port}\033[0m")
        try:
            await manage_connections()
        except Exception as e:
            print("Error in manage_connections")
            pass


if __name__ == "__main__":
    try:
        asyncio.run(run_ws_server())
    except KeyboardInterrupt:
        print("Server stopped by user.")
