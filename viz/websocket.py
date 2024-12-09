import asyncio
from collections import defaultdict
import json
import numpy as np
import websockets
from websockets.asyncio.server import ServerConnection

connections = defaultdict[str, set[ServerConnection]]()
connections.default_factory = lambda: set()


async def send_data(path: str, data: dict):
    conn_set = connections.get(path)
    if not conn_set:
        return
    for conn in conn_set:
        await conn.send(json.dumps(data))


async def handle_connection(conn: ServerConnection):
    path = conn.request.path
    print(f"Connection established at path: {path}")
    connections[path].add(conn)
    try:
        await conn.wait_closed()
        connections[path].remove(conn)
    except websockets.exceptions.ConnectionClosedOK:
        print("Connection closed normally")
    except Exception as e:
        print(f"An error occurred: {e}")


async def update_particles():
    particles = np.random.multivariate_normal(np.array([0., 0.]),
                                              np.array([[1., 0.], [0., 1.]]),
                                              size=1000)
    weight = np.random.uniform(0., 1., 1000)
    data = {
        "x": particles[:, 0].tolist(),
        "y": particles[:, 1].tolist(),
        "weight": weight.tolist()
    }
    await send_data("/particles", data)


async def particle_updater():
    while True:
        await update_particles()
        await asyncio.sleep(1)


async def start_ws_server():
    print("Starting websocket server")

    async with websockets.serve(handle_connection, "127.0.0.1", 8766):
        await particle_updater()  # run forever


def cleanup_ws():
    pass
