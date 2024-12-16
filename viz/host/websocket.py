import asyncio
from collections import defaultdict
import json
import websockets
from websockets.asyncio.server import ServerConnection


class WebsocketServer:

    def __init__(self, port: int = 8766, host: str = "127.0.0.1"):
        self.connections = defaultdict[str, set[ServerConnection]]()
        self.connections.default_factory = lambda: set()
        self.port = port
        self.host = host

    async def send_data(self, path: str, data: dict):
        conn_set = self.connections.get(path)
        if not conn_set:
            return
        for conn in conn_set:
            await conn.send(json.dumps(data))
        # print(data.keys())

    def print_connections(self):
        print("Connections:")
        for path, conns in self.connections.items():
            print(f"{path}: {len(conns)} connections")

    async def handle_connection(self, conn: ServerConnection):
        path = conn.request.path
        print(f"Connection established at path: {path}")
        self.connections[path].add(conn)
        try:
            await conn.wait_closed()
            self.connections[path].remove(conn)
        except websockets.exceptions.ConnectionClosedOK:
            print("Connection closed normally")
        except Exception as e:
            print(f"An error occurred: {e}")

    async def start(self):
        print("Starting websocket server")

        async with websockets.serve(self.handle_connection, self.host,
                                    self.port):
            await asyncio.Future()  # run forever

    async def cleanup(self):
        for conns in self.connections.values():
            for conn in conns:
                await conn.close()
