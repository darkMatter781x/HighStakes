from multiprocessing.connection import Listener as ConnListener
from parser import Handler, Particle


class PacketHandler(Handler):
    def __init__(self,
                 port: int = 8765,
                 host: str = "localhost",
                 authkey: bytes = b''):
        self.listener = ConnListener((host, port), authkey=authkey)
        self.listener.accept()
        
