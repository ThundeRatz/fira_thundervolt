from .receiver import Receiver
from .protocols import vssref_command_pb2

import json
from google.protobuf.json_format import MessageToJson

class RefereeComm(Receiver):
    def __init__(self, referee_ip='224.0.0.1', referee_port=10003):
        super(RefereeComm, self).__init__(referee_ip, referee_port)

    def receive(self):
        """
        Receive packet and decode.
        """

        data = super().receive()
        decoded_data = vssref_command_pb2.VSSRef_Command().FromString(data)

        referee_data = json.loads(MessageToJson(decoded_data))

        return referee_data
