import asyncio
import os
import signal
import subprocess
from typing import Callable, Coroutine, override

from . import Target

class V5Brain(Target):
    """
    Runs program in vexide sim and provides access to the raw output
    """

    def __init__(
        self,
    ):
        self.command = "cargo v5 run --file ./bin/sim/monolith.bin -s 7 -i question-mark"
        self.process = None

    @override
    async def run(self, on_receive_line: Callable[[str], None]):
        """ 
        Runs the program on the v5 brain and calls on_receive_line for each line of output
        """

        if self.process is not None:
            raise Exception("Program is already running")

        self.process = await asyncio.create_subprocess_shell(
            self.command,
            shell=True,
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE)
        while True:
            line = (await self.process.stdout.readline()).decode()
            if line == '' and self.process.returncode is not None:
                break
            await on_receive_line(line)
        print("Program finished??")

    def cleanup(self):
        if self.process is not None and self.process.returncode is None:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
