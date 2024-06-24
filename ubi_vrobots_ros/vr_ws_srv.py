import asyncio
import websockets
from .vr_msg_py.EMPT_empty_generated import EmptyMsg
from .vr_msg_py.R000_states_generated import StatesMsg

CONNECTIONS = set()
CMDS_DICT = {}


async def register(websocket, ros_publisher_cb):
    CONNECTIONS.add(websocket)
    # print("new connection")
    try:
        async for message in websocket:
            # print(f"message: {message}")
            if EmptyMsg.EmptyMsgBufferHasIdentifier(message, 0) is True:
                continue
            if message == b"ping" or message == "ping":
                await websocket.send(b"pong")
                continue
            # print(f"message: {message}")
            # others = CONNECTIONS - {websocket}
            if (StatesMsg.StatesMsgBufferHasIdentifier(message, 0) is True):
                ros_publisher_cb(message)
            #websockets.broadcast(others, message)
            # websocket.send("ack")



            

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
