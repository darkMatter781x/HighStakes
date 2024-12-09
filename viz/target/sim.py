import os
import signal
import subprocess
from typing import Callable

from target import Target


class Simulator(Target):
    """
    Runs program in vexide sim and provides access to the raw output
    """

    def __init__(
        self,
        # Path to the simulator executable
        sim_path: str = "simulator",
        # Subcommand for the simulator
        sim_cmd: str = "monolith",
        # Path to the binary to run
        bin_path: str = "./bin/sim/monolith.bin",
        # Arguments to pass to the simulator
        args: list[str] = [],
        # Overrides everything else if set, and runs the given command
        command: str = None,
    ):
        self.command = " ".join([sim_path, sim_cmd, bin_path] + args)
        if command is not None:
            self.command = command
        self.process = None

    def run(self, on_receive_line: Callable[[str], None]):
        """ 
        Runs the vexide sim and calls on_receive_line for each line of output
        """

        if self.process is not None:
            raise Exception("Simulator is already running")

        self.process = subprocess.Popen(self.command,
                                        shell=True,
                                        preexec_fn=os.setsid,
                                        text=True,
                                        stdout=subprocess.PIPE)
        while True:
            line = self.process.stdout.readline()
            if line == '' and self.process.poll() is not None:
                break
            on_receive_line(line)
        print("Simulation finished??")

    def cleanup(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
