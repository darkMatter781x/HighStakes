import asyncio
import json
from typing import Generator, override

import numpy as np
from host.parser import Handler, Particle, ParticleTelemetry, Point, Pose
from host.websocket import WebsocketServer

import rerun as rr

rr.init("781x-PF", spawn=False)
rr.connect_tcp()


class PacketHandler(Handler):

    def __init__(self, ws_server: WebsocketServer):
        self.ws_server: WebsocketServer = ws_server

    async def _send_dict(self, topic: str, data: dict):
        await self.ws_server.send_data("/data", {topic: data})

    @override
    async def update_particles(self, particleTelemetry: ParticleTelemetry):
        data = {"x": [], "y": [], "weight": []}
        particleList = list(particleTelemetry[1])
        particleList.sort(key=lambda p: p["weight"], reverse=True)

        for particle in particleList[:]:
            data["x"].append(particle["x"])
            data["y"].append(particle["y"])
            data["weight"].append(particle["weight"])

        await self._send_dict("particles", data)
        # rr.set_time_seconds("capture_time", particleTelemetry[0])
        # data = [[p["x"], p["y"]] for p in particleTelemetry[1]]
        # print(f"sending particles: {data}")

        # rr.log(
        #     "pf/particles",
        #     rr.Points2D(data, radii=0.01),
        # )

    @override
    async def update_calculated_pose(self, pose: Pose):
        # print(f"updated calculated pose: {pose}")
        await self._send_dict("calculated_pose", pose)

    @override
    async def update_actual_pose(self, pose: Pose):
        # print(f"updated actual pose: {pose}")
        await self._send_dict("actual_pose", pose)

    @override
    async def update_lasers(self, laser_data: tuple[int, Point, Point]):
        # print(f"updated lasers pose: {laser_data}")
        await self._send_dict("lasers", laser_data)
        # print(f"updated lasers: {laser_data}")

    @override
    async def update_raw_odom_pose(self, pose: Pose):
        # print(f"updated raw odom pose: {pose}")
        await self._send_dict("raw_odom_pose", pose)

    def cleanup(self):
        pass
