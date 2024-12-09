import asyncio
import atexit
from threading import Thread

from app import start_web_server, cleanup_app
from parser import Parser
from websocket import WebsocketServer
from target.sim import Simulator
from build_project import build
from multiprocessing import Process

cleanup_funcs = [cleanup_app]


def cleanup(_=None):
    for func in cleanup_funcs:
        try:
            func()
        except Exception as e:
            print(f"Error during cleanup: {e}")


# Build the project
build("sim")

# Handle cleanup
atexit.register(cleanup)

sim = Simulator()

ws_server = WebsocketServer()
cleanup_funcs.append(ws_server.cleanup)

parser=Parser(listener=ws_server)

def run_dash_app():
    start_web_server()


async def main():
    # Start the websocket server
    server_task = asyncio.create_task(ws_server.start())
    # Start the web server
    dash_process = Process(target=run_dash_app)

    sim_thread = Thread(target=sim.run, args=(parser.parse, ))

    dash_process.start()
    sim_thread.start()

    def cleanup_processes():
        dash_process.terminate()
        sim.cleanup()
        dash_process.join()
        sim_thread.join()

    cleanup_funcs.append(cleanup_processes)

    await server_task


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Shutting down...")
    cleanup()
