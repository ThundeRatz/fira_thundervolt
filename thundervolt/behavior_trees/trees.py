import py_trees

from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.conditions import xPlayerLTd
from thundervolt.behavior_trees.nodes.action.back_to_goal import BackToGoal
from thundervolt.behavior_trees.nodes.conditions import BallDistanceToDefenseAreaLTd
from thundervolt.behavior_trees.nodes.action.boost import BoostBall
from thundervolt.behavior_trees.nodes.action.clear_ball import ClearBall
from thundervolt.behavior_trees.nodes.conditions import PlayerDistanceToDefenseAreaLTd
from thundervolt.behavior_trees.nodes.action.save import SaveGoal
from thundervolt.behavior_trees.nodes.action.follow_ball_y import FollowBallVertical


def CreateGoalkeeperTree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    back_far_condition = xPlayerLTd("Back From Far Condition", "/goalkeeper", field_data, -0.4)
    back_far_inverter = py_trees.decorators.Inverter("Back From Close Inverter", back_far_condition)
    back_far_action = BackToGoal("Back from Far Action", "/goalkeeper", field_data, team_command)
    back_far_node = py_trees.composites.Sequence("Back From Far Node", [back_far_inverter, back_far_action])
    root.add_child(back_far_node)

    spin_condition = BallDistanceToDefenseAreaLTd("Spin Condition", "/goalkeeper", field_data, 0.1)
    boost_action = BoostBall("Boost Action Node", "/goalkeeper", field_data, team_command)
    clear_action = ClearBall("Clear Ball Action Node", "/goalkeeper", field_data, team_command)
    spin_node = py_trees.composites.Sequence("Spin Node", [spin_condition, boost_action, clear_action])
    root.add_child(spin_node)

    back_close_condition = PlayerDistanceToDefenseAreaLTd("Back From Close Condition", "/goalkeeper", field_data, 0.0)
    back_close_inverter = py_trees.decorators.Inverter("Back From Close Inverter", back_close_condition)
    back_close_action = BackToGoal("Back From Close Action", "/goalkeeper", field_data, team_command)
    back_close_node = py_trees.composites.Sequence("Back From Close Node", [back_close_inverter, back_close_action])
    root.add_child(back_close_node)

    save_node = SaveGoal("Save Node", "/goalkeeper", field_data, team_command, 3.0)
    root.add_child(save_node)

    follow_node = FollowBallVertical("Follow Node", "/goalkeeper", field_data, team_command, x_position=-0.7,
                                                                        limit_sup=0.18, limit_inf=-0.18)
    root.add_child(follow_node)

    return root

def CreateDefenderTree():
    pass

def CreateStrikerTree():
    pass
