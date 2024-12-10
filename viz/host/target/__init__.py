from typing import Callable

class Target:
    """
    Runs program and provides access to the raw output
    """

    def __init__(self):
        pass

    def run(self, on_receive_line: Callable[[str], None]):
        """ 
        Runs the target and calls on_receive_line for each line of output
        """
        raise NotImplementedError

    def cleanup(self):
        """
        Cleans up the target
        """
        raise NotImplementedError

from .sim import Simulator