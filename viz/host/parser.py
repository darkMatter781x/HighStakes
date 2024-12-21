# Parses program output

from collections import namedtuple
import re
from typing import Generator, TypedDict


class Point(TypedDict):
    x: float
    y: float


class Pose(Point):
    theta: float


class Particle(Point):
    weight: float


class Pose(Point):
    theta: float


class Handler:
    """
    Listens to output from the parser and updates the visualizer
    """

    async def update_particles(self, particles: Generator[Particle, None,
                                                          None]):
        raise NotImplementedError

    async def update_calculated_pose(self, pose: Pose):
        raise NotImplementedError

    async def update_actual_pose(self, pose: Pose):
        raise NotImplementedError

    async def update_raw_odom_pose(self, pose: Pose):
        raise NotImplementedError

    async def update_lasers(self, laser_data: tuple[int, Point, Point]):
        raise NotImplementedError


class Parser:

    class PacketType:

        def __init__(self,
                     identifier=None,
                     shouldPrint=True,
                     handler=None,
                     matcher=None,
                     removeIdentifier=True):
            if identifier is not None:
                self.startPattern = "[{identifier}]:".format(
                    identifier=identifier)
            else:
                self.startPattern = None
            self.shouldPrint = shouldPrint
            self.handler = handler
            self.matcher = matcher
            self.removeIdentifier = removeIdentifier

        def matches(self, line):
            if self.matcher is not None:
                return self.matcher(line)
            if self.startPattern is None:
                return True
            return line.startswith(self.startPattern)

        async def handle(self, line):
            if self.shouldPrint:
                print(line, flush=True, end="")
            if self.handler is not None:
                if self.startPattern is not None and self.removeIdentifier:
                    await self.handler(line[len(self.startPattern):])
                else:
                    await self.handler(line)

    def __init__(self, handler: Handler):
        self.packet_types = [
            Parser.PacketType(
                "TELEMETRY.PF.PARTICLES",
                handler=(lambda dataStr: handler.update_particles(
                    parse_particles(dataStr))),
                shouldPrint=False),
            Parser.PacketType(
                "TELEMETRY.PF.CALC_POSE",
                handler=(lambda dataStr: handler.update_calculated_pose(
                    parse_pose(dataStr))),
                shouldPrint=False),
            Parser.PacketType(
                "TELEMETRY.PF.ACTUAL_POSE",
                handler=(lambda dataStr: handler.update_actual_pose(
                    parse_pose(dataStr))),
                shouldPrint=False),
            Parser.PacketType(
                "TELEMETRY.PF.RAW_ODOM_POSE",
                handler=(lambda dataStr: handler.update_raw_odom_pose(
                    parse_pose(dataStr))),
                shouldPrint=False),
            Parser.PacketType("TELEMETRY.PF.LASERS",
                              handler=(lambda dataStr: handler.update_lasers(
                                  parse_lasers(dataStr))),
                              shouldPrint=False),
            # Matches all and prints all
            Parser.PacketType(),
        ]

    async def parse(self, line: str):
        """
        Parses a line of output from the program, and calls the appropriate listener method
        """
        for type in self.packet_types:
            if type.matches(line):
                try:
                    await type.handle(line)
                except Exception as e:
                    print("Error handling packet: ", e)
                return


def split_iter(string, sep):
    # from: https://stackoverflow.com/a/9770397
    # warning: does not yet work if sep is a lookahead like `(?=b)`
    if sep == '':
        return (c for c in string)
    else:
        return (_.group(1)
                for _ in re.finditer(f'(?:^|{sep})((?:(?!{sep}).)*)', string))


def parse_particles(dataStr: str) -> Generator[Particle, None, None]:
    # dataStr is formatted like:
    # "<x_float_1>,<y_float_1>,<weight_float_1>;<x_float_2>,<y_float_2>,<weight_float_2>;..."
    for particle in split_iter(dataStr, ";"):
        coords = particle.split(",")
        if len(coords) == 1 and coords[0].strip() == "":
            # Should be end of string
            continue
        if len(coords) != 3:
            raise ValueError("Invalid particle telemetry: ", particle)
        x_new, y_new, weight_new = coords
        yield Particle(x=float(x_new),
                       y=float(y_new),
                       weight=float(weight_new))


def parse_pose(dataStr: str) -> Pose:
    # dataStr is formatted like:
    # "<x_float>,<y_float>,<theta_float>"
    data = dataStr.split(",")
    if len(data) != 3:
        raise ValueError("Invalid pose telemetry: ", dataStr)
    x, y, theta = data
    return Pose(x=float(x), y=float(y), theta=float(theta))


def parse_lasers(dataStr: str) -> tuple[int, Point, Point]:
    # dataStr is formatted like:
    # "id,<start_x>,<start_y>,<end_x>,<end_y>;<start_x>,<start_y>,<end_x>,<end_y>;..."
    data = dataStr.split(",")
    if len(data) != 5:
        raise ValueError("Invalid laser telemetry: ", dataStr)
    id, start_x, start_y, end_x, end_y = data
    return (id, Point(x=float(start_x),
                      y=float(start_y)), Point(x=float(end_x), y=float(end_y)))
