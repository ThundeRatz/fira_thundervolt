import py_trees

from thundervolt.core import data
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.action.back_to_goal import BackToGoal
from thundervolt.behavior_trees.nodes.action.clear_ball import ClearBall
from thundervolt.behavior_trees.nodes.action.save import SaveGoal
from thundervolt.behavior_trees.nodes.action.follow_ball_y import FollowBallVertical
from thundervolt.behavior_trees.nodes.action.go_to_foes_goal import GoToFoesGoal
from thundervolt.behavior_trees.nodes.action.get_ball import GetBall
from thundervolt.behavior_trees.nodes.action.wait_to_strike import WaitToStrike
from thundervolt.behavior_trees.nodes.conditions import *

PLAYER_DIST_TO_GOAL = 2.2 * data.ROBOT_SIZE
BALL_DIST_TO_PLAYER = 0.75 * data.ROBOT_SIZE + data.BALL_RADIUS
BALL_DIST_TO_GOAL = 1.25 * data.ROBOT_SIZE + data.BALL_RADIUS
AREA_LINE_X = data.FIELD_LENGTH/2 - data.GOAL_AREA_DEPTH
CLOSE_LINE_Y = data.GOAL_AREA_WIDTH/2
LINE_X = data.FIELD_LENGTH/2 - 0.8 * data.ROBOT_SIZE
SPIN_LINE = LINE_X -data.ROBOT_SIZE/2
DEFENDER_AREA_X = -data.FIELD_LENGTH/2

def create_goalkeeper_tree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    back_condition = PlayerDistToGoalLTd("Back Condition", "/goalkeeper", field_data, PLAYER_DIST_TO_GOAL)
    back_inverter = py_trees.decorators.Inverter(name="Back Inverter", child=back_condition)
    back_action = BackToGoal("Back Action", "/goalkeeper", field_data, team_command)
    back_parallel = py_trees.composites.Parallel(name="Back Parallel", children=[back_inverter, back_action])
    root.add_child(back_parallel)

    save_node = SaveGoal("Save Node", "/goalkeeper", field_data, team_command, 3.0, LINE_X, CLOSE_LINE_Y)
    root.add_child(save_node)

    spin_condition1 = BallDistToPlayerLTd("Spin Condition 1", "/goalkeeper", field_data, BALL_DIST_TO_PLAYER)
    spin_condition2 = BallDistToGoalLTd("Spin Condition 2", "/goalkeeper", field_data, BALL_DIST_TO_GOAL)
    spin_condition3 = xBallLTd("Spin Condition 3", "/goalkeeper", field_data, -SPIN_LINE)
    spin_inverter = py_trees.decorators.Inverter(name="Spin Inverter", child=spin_condition2)
    clear_action = ClearBall("Clear Ball Action", "/goalkeeper", field_data, team_command)
    spin_parallel = py_trees.composites.Parallel(name="Spin Parallel", children=[spin_condition1, spin_inverter, spin_condition3, clear_action])
    root.add_child(spin_parallel)

    follow_close_condition = xBallLTd("Follow Close Condition", "/goalkeeper", field_data, -AREA_LINE_X)
    follow_close_action = FollowBallVertical("Follow Node", "/goalkeeper", field_data, team_command,
                                                x_position=-LINE_X, limit_sup=CLOSE_LINE_Y, limit_inf=-CLOSE_LINE_Y)
    follow_close_parallel = py_trees.composites.Parallel(name="Follow Close Parallel", children=[follow_close_condition, follow_close_action])
    root.add_child(follow_close_parallel)

    follow_far_node = FollowBallVertical("Follow Far Node", "/goalkeeper", field_data, team_command,
                                                x_position=-LINE_X, limit_sup=data.GOAL_WIDTH/2, limit_inf=-data.GOAL_WIDTH/2)
    root.add_child(follow_far_node)

    return root

def CreateDefenderTree(field_data, team_command):
    pass

def create_striker_tree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    strike_condition_one = xPlayerLTxBall("Strike Condition 1", "/striker", field_data)
    strike_condition_two = BallDistToPlayerLTd("Strike Condition 2", "/striker", field_data, BALL_DIST_TO_PLAYER)
    strike_condition_three = GoodStrikerOrientation("Strike Condition 3", "/striker", field_data, np.pi/6, 0.0)
    strike_action = GoToFoesGoal("Strike Action", "/striker", field_data, team_command, BALL_DIST_TO_PLAYER)
    strike_parallel = py_trees.composites.Parallel(name="Strike Parallel", children=[
        strike_condition_one, strike_condition_two, strike_condition_three, strike_action
    ])
    root.add_child(strike_parallel)
    
    get_ball_condition_one = xPlayerLTxBall("Get Ball Condition 1", "/striker", field_data)
    get_ball_condition_two = xBallLTd("Get Ball Condition 2", "/striker", field_data, 0.0)
    get_ball_inverter = py_trees.decorators.Inverter(name="Get Ball Inverter", child=get_ball_condition_two)
    get_ball_selector = py_trees.composites.Selector(name="Get Ball Selector", children=[
        get_ball_condition_one, get_ball_inverter
    ])
    get_ball_action = GetBall("Get Ball Action", "/striker", field_data, team_command, 0.0)
    #get_ball_parallel = py_trees.composites.Parallel(name="Get Ball Node", children=[get_ball_condition_one, get_ball_inverter, get_ball_action])
    get_ball_parallel = py_trees.composites.Parallel(name="Get Ball Node", children=[get_ball_selector, get_ball_action])
    root.add_child(get_ball_parallel)

    wait_to_strike_node = WaitToStrike("Wait to Strike Node", "/striker", field_data, team_command, 0.0)
    root.add_child(wait_to_strike_node)

    return root
