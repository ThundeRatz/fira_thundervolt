import socket
from abc import ABC, abstractmethod

class Receiver(ABC):
    def __init__(self, receiver_ip='224.0.0.1', receiver_port=10002):
        """
        Init Client object.
        Extended description of function.
        Parameters
        ----------
        ip : str
            Multicast IP in format '255.255.255.255'.
        port : int
            Port up to 1024.
        """

        self.receiver_ip = receiver_ip
        self.receiver_port = receiver_port

        # Create socket
        self.receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.receiver_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.receiver_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.receiver_socket.bind((self.receiver_ip, self.receiver_port))

    @abstractmethod
    def receive(self):
        """
        Receive packet.
        """

        data, _ = self.receiver_socket.recvfrom(1024)

        return data
