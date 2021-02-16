from .transmitter import Transmitter
from .protocols import packet_pb2
from .protocols import command_pb2

class FiraControl(Transmitter):
    def __init__(self, team_color_yellow=True, control_ip='127.0.0.1', control_port=20011):
        super(FiraControl, self).__init__(control_ip, control_port)

        self.team_color_yellow = team_color_yellow

    def transmit(self, robot_id, left_speed, right_speed):
        """
        Encode package and transmit.
        """

        packet = self._fill_command_packet(robot_id, left_speed, right_speed)

        super().transmit(packet)

    def _fill_command_packet(self, robot_id, left_speed, right_speed):
        cmd_packet = command_pb2.Commands()

        robot = cmd_packet.robot_commands.add()
        robot.id          = robot_id
        robot.yellowteam  = self.team_color_yellow
        robot.wheel_left  = left_speed
        robot.wheel_right = right_speed

        packet = packet_pb2.Packet()
        packet.cmd.CopyFrom(cmd_packet)

        return packet
