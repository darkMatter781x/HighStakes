import asyncio
import atexit
from threading import Thread

from host.app import start_web_server, cleanup_app
from host.parser import Parser
from host.websocket import WebsocketServer
from host.target.sim import Simulator
from host.build_project import build
from host.handler import PacketHandler
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

packet_handler = PacketHandler(ws_server)
cleanup_funcs.append(packet_handler.cleanup)

parser = Parser(handler=packet_handler)


# def run_dash_app():
#     start_web_server()
async def main():
    # Start the websocket server
    ws_server_coroutine = ws_server.start()
    # Start the web server
    # dash_process = Process(target=run_dash_app)

    sim_coroutine = sim.run(parser.parse)

    # # dash_process.start()

    def cleanup_processes():
        # dash_process.terminate()
        sim.cleanup()
        # dash_process.join()
        # sim_coroutine.join()
        pass

    cleanup_funcs.append(cleanup_processes)

    await asyncio.gather(ws_server_coroutine, sim_coroutine)


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Shutting down...")
    cleanup()
