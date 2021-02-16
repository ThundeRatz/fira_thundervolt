from .receiver import Receiver
from .protocols import packet_pb2

class FiraVision(Receiver):
    def __init__(self, vision_ip='224.0.0.1', vision_port=10002):
        super(FiraVision, self).__init__(vision_ip, vision_port)

    def receive(self):
        """
        Receive packet and decode.
        """

        data = super().receive()
        decoded_data = packet_pb2.Environment().FromString(data)
        vision_frame = decoded_data.frame

        return vision_frame
