import py_trees

from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.conditions import PlayerDistToGoalLTd
from thundervolt.behavior_trees.nodes.action.back_to_goal import BackToGoal
from thundervolt.behavior_trees.nodes.conditions import BallDistToPlayerLTd
from thundervolt.behavior_trees.nodes.conditions import BallDistToGoalLTd
from thundervolt.behavior_trees.nodes.action.clear_ball import ClearBall
from thundervolt.behavior_trees.nodes.action.save import SaveGoal
from thundervolt.behavior_trees.nodes.conditions import xBallLTd
from thundervolt.behavior_trees.nodes.action.follow_ball_y import FollowBallVertical

def create_goalkeeper_tree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    back_condition = PlayerDistToGoalLTd("Back Condition", "/goalkeeper", field_data, 0.1125)
    back_inverter = py_trees.decorators.Inverter(name="Back Inverter", child=back_condition)
    back_action = BackToGoal("Back Action", "/goalkeeper", field_data, team_command)
    back_node = py_trees.composites.Parallel(name="Back Node", children=[back_inverter, back_action])
    root.add_child(back_node)

    after_save_condition = BallDistToPlayerLTd("After Save Condition", "/goalkeeper", field_data, 0.075)
    after_save_action = ClearBall("After Save Action", "/goalkeeper", field_data, team_command)
    after_save_node = py_trees.composites.Parallel(name="After Save Node", children=[after_save_condition, after_save_action])
    save_action = SaveGoal("Save Node", "/goalkeeper", field_data, team_command, 3.0)
    save_node = py_trees.composites.Sequence(name="Save Node", children=[save_action, after_save_node])
    root.add_child(save_node)

    spin_condition1 = BallDistToPlayerLTd("Spin Condition 1", "/goalkeeper", field_data, 0.075)
    spin_condition2 = BallDistToGoalLTd("Spin Condition 2", "/goalkeeper", field_data, 0.0588)
    spin_inverter = py_trees.decorators.Inverter(name="Spin Inverter", child=spin_condition2)
    clear_action = ClearBall("Clear Ball Action", "/goalkeeper", field_data, team_command)
    spin_node = py_trees.composites.Parallel(name="Spin Node", children=[spin_condition1, spin_inverter, clear_action])
    root.add_child(spin_node)

    follow_close_condition = xBallLTd("Follow Close Condition", "/goalkeeper", field_data, -0.6)
    follow_close_action = FollowBallVertical("Follow Node", "/goalkeeper", field_data, team_command,
                                                x_position=-0.675, limit_sup=0.275, limit_inf=-0.275)
    follow_close_node = py_trees.composites.Parallel(name="Follow Close Node", children=[follow_close_condition, follow_close_action])
    root.add_child(follow_close_node)

    follow_far_node = FollowBallVertical("Follow Far Node", "/goalkeeper", field_data, team_command,
                                                x_position=-0.675, limit_sup=0.2, limit_inf=-0.2)
    root.add_child(follow_far_node)

    return root

def CreateDefenderTree():
    pass

def CreateStrikerTree():
    pass
