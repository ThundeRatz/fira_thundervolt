import py_trees

from thundervolt.core import data
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

PLAYER_DIST_TO_GOAL = 1.5 * data.ROBOT_SIZE
BALL_DIST_TO_PLAYER = 0.75 * data.ROBOT_SIZE + data.BALL_RADIUS
BALL_DIST_TO_GOAL = 1.5 * data.ROBOT_SIZE + data.BALL_RADIUS
AREA_LINE_X = data.FIELD_LENGTH/2 - data.GOAL_AREA_DEPTH
CLOSE_LINE_Y = data.GOAL_AREA_WIDTH/2 - data.ROBOT_SIZE/2
LINE_X = data.FIELD_LENGTH/2 - 0.75 * data.ROBOT_SIZE

def create_goalkeeper_tree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    back_condition = PlayerDistToGoalLTd("Back Condition", "/goalkeeper", field_data, PLAYER_DIST_TO_GOAL)
    back_inverter = py_trees.decorators.Inverter(name="Back Inverter", child=back_condition)
    back_action = BackToGoal("Back Action", "/goalkeeper", field_data, team_command)
    back_node = py_trees.composites.Parallel(name="Back Node", children=[back_inverter, back_action])
    root.add_child(back_node)

    after_save_condition = BallDistToPlayerLTd("After Save Condition", "/goalkeeper", field_data, BALL_DIST_TO_PLAYER)
    after_save_action = ClearBall("After Save Action", "/goalkeeper", field_data, team_command)
    after_save_node = py_trees.composites.Parallel(name="After Save Node", children=[after_save_condition, after_save_action])
    save_action = SaveGoal("Save Node", "/goalkeeper", field_data, team_command, 3.0)
    save_node = py_trees.composites.Sequence(name="Save Node", children=[save_action, after_save_node])
    root.add_child(save_node)

    spin_condition1 = BallDistToPlayerLTd("Spin Condition 1", "/goalkeeper", field_data, BALL_DIST_TO_PLAYER)
    spin_condition2 = BallDistToGoalLTd("Spin Condition 2", "/goalkeeper", field_data, BALL_DIST_TO_GOAL)
    spin_inverter = py_trees.decorators.Inverter(name="Spin Inverter", child=spin_condition2)
    clear_action = ClearBall("Clear Ball Action", "/goalkeeper", field_data, team_command)
    spin_node = py_trees.composites.Parallel(name="Spin Node", children=[spin_condition1, spin_inverter, clear_action])
    root.add_child(spin_node)

    follow_close_condition = xBallLTd("Follow Close Condition", "/goalkeeper", field_data, -AREA_LINE_X)
    follow_close_action = FollowBallVertical("Follow Node", "/goalkeeper", field_data, team_command,
                                                x_position=-LINE_X, limit_sup=CLOSE_LINE_Y, limit_inf=-CLOSE_LINE_Y)
    follow_close_node = py_trees.composites.Parallel(name="Follow Close Node", children=[follow_close_condition, follow_close_action])
    root.add_child(follow_close_node)

    follow_far_node = FollowBallVertical("Follow Far Node", "/goalkeeper", field_data, team_command,
                                                x_position=-LINE_X, limit_sup=data.GOAL_WIDTH/2, limit_inf=-data.GOAL_WIDTH/2)
    root.add_child(follow_far_node)

    return root

def CreateDefenderTree():
    pass

def CreateStrikerTree():
    pass
