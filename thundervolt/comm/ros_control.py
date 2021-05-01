import logging

import rospy
from std_msgs.msg import Float64
from ..core.command import TeamCommand

class TraveSimControl():
    def __init__(self, team_color_yellow: bool, team_command: TeamCommand = None):

        self.team_color_yellow = team_color_yellow
        self.team_command = team_command

        left_topics_names = []
        right_topics_names = []

        if (self.team_color_yellow):
            team_namespace = "/yellow_team"
        else:
            team_namespace = "/blue_team"

        for robot_id in range(3):
            left_name = team_namespace + f"/robot_{robot_id}/left_controller/command"
            left_topics_names.append(left_name)
            right_name = team_namespace + f"/robot_{robot_id}/right_controller/command"
            right_topics_names.append(right_name)

        self.left_pubs = [rospy.Publisher(topic_name, Float64, queue_size=1) for topic_name in left_topics_names]
        self.right_pubs = [rospy.Publisher(topic_name, Float64, queue_size=1) for topic_name in right_topics_names]


    def transmit_robot(self, robot_id, left_vel, right_vel):
        """
        Encode package and transmit.
        """

        self.left_pubs[robot_id].publish(Float64(left_vel))
        self.right_pubs[robot_id].publish(Float64(right_vel))


    def transmit_team(self, team_cmd : TeamCommand):
        """
        Encode package and transmit.

        Parameters
        ----------
        team_cmd : core.commands.TeamCommand
            Commands to all robots
        """

        for i in range(len(team_cmd.commands)):
            self.transmit_robot(i, team_cmd.commands[i].left_speed, team_cmd.commands[i].right_speed)


    def update(self):
        """
        Update the transmitted packet with the team_command
        passed in the constructor
        """

        if self.team_command is None:
            logging.error('TeamCommand not instantiated', exc_info=True)
        else:
            self.transmit_team(self.team_command)


    def stop_team(self):
        stop_team_cmd = TeamCommand()
        self.transmit_team(stop_team_cmd)
