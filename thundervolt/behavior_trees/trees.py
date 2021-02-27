import py_trees

from thundervolt.core import data
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.conditions import *
from thundervolt.behavior_trees.nodes.action.save import SaveGoal
from thundervolt.behavior_trees.nodes.action.get_ball import GetBall
from thundervolt.behavior_trees.nodes.action.clear_ball import ClearBall
from thundervolt.behavior_trees.nodes.action.back_to_goal import BackToGoal
from thundervolt.behavior_trees.nodes.action.look_at_ball import LookAtBall
from thundervolt.behavior_trees.nodes.action.defend_corner import DefendCorner
from thundervolt.behavior_trees.nodes.action.wait_to_strike import WaitToStrike
from thundervolt.behavior_trees.nodes.action.go_near_corner import GoNearCorner
from thundervolt.behavior_trees.nodes.action.go_behind_ball import GoBehindBall
from thundervolt.behavior_trees.nodes.action.go_to_foes_goal import GoToFoesGoal
from thundervolt.behavior_trees.nodes.action.go_back_goal_area import BackToGoalArea
from thundervolt.behavior_trees.nodes.action.get_ball_defender import GetBallDefender
from thundervolt.behavior_trees.nodes.action.follow_ball_vertical import FollowBallVertical
from thundervolt.behavior_trees.nodes.action.follow_ball_horizontal import FollowBallHorizontal
from thundervolt.behavior_trees.nodes.action.follow_ball_vertical_defender import FollowBallVerticalDefender

PLAYER_DIST_TO_GOAL = 2.2 * data.ROBOT_SIZE                         # Max distance from the goal to the goalkeeper go back
BALL_DIST_TO_PLAYER = 0.75 * data.ROBOT_SIZE + data.BALL_RADIUS     # Minimal distance from ball for striker to attack
BALL_DIST_TO_GOAL = 1.25 * data.ROBOT_SIZE + data.BALL_RADIUS       # Maximum ball distance to goal to spin
AREA_LINE_X = data.FIELD_LENGTH/2 - data.GOAL_AREA_DEPTH            # Goal Area X Line
CLOSE_LINE_Y = data.GOAL_AREA_WIDTH/2                               # X Line for follow action node when near ball
LINE_X = data.FIELD_LENGTH/2 - 0.8 * data.ROBOT_SIZE                # X Line for follow action node when far from the ball
SPIN_LINE = LINE_X - data.ROBOT_SIZE/2                              # Max x position the ball can reach before the goalkeeper spins

DEFENDER_CORNER_Y = 0.4
DEFENDER_AREA_X = -data.FIELD_LENGTH/6                              # Minimal x position that the striker can reach
DEFENDER_LINE = -0.52                                               # Line position that the defender will follow in front of the area


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


def create_defender_tree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    ''' Ball in defense '''
    # Ball in defense node
    ball_in_defense_condition = xBallLTd("Ball in defense condition", "/defender", field_data, 0.0)
    ## Ball in defense actions
    ### Ball near goal node
    ball_near_goal_condition = xBallLTd("Ball x in defense goal area", "/defender", field_data, -data.FIELD_LENGTH/2 + data.GOAL_AREA_DEPTH + 0.05)  # plus a 5 cm tolerance

    #### Corner node
    go_near_corner_action = GoNearCorner("Go near corner action", "/defender", field_data, team_command, y_position = DEFENDER_CORNER_Y)
    ##### After go to corner node
    ###### Spin node
    spin_condition = BallDistToPlayerLTd("Spin Condition", "/defender", field_data, BALL_DIST_TO_PLAYER)
    spin_action = ClearBall("Clear Ball Action", "/defender", field_data, team_command)
    spin_node = py_trees.composites.Parallel(name="Spin Node", children=[spin_condition, spin_action])
    ##### Spin node
    ###### Defend corner node
    defend_corner_action = DefendCorner("Defend corner action", "/defender", field_data, team_command, y_position = DEFENDER_CORNER_Y)
    ###### Defend corner node
    after_go_corner_node = py_trees.composites.Selector("After go to corner node", [spin_node, defend_corner_action])
    ##### After go to corner node
    corner_node = py_trees.composites.Sequence(name="Corner node", children=[go_near_corner_action, after_go_corner_node])
    #### Corner node
    ball_near_goal_node = py_trees.composites.Parallel(name="Ball near goal node", children=[ball_near_goal_condition, corner_node])
    ### Ball near goal node

    ### Get ball node
    ball_x_lt_limit_striker_condition = xBallLTd("Ball x less than limit striker", "/defender", field_data, d_position = DEFENDER_AREA_X)
    get_ball_action = GetBallDefender("Get ball action", "/defender", field_data, team_command, min_position = DEFENDER_LINE - 0.05) # minus a 5 cm tolerance
    get_ball_node = py_trees.composites.Parallel("Get ball node", children=[ball_x_lt_limit_striker_condition, get_ball_action])
    ### Get ball node

    ### Assistant follow ball y node
    assistant_go_back = BackToGoalArea("Go back to area line action", "/defender", field_data, team_command, DEFENDER_LINE)
    assistant_follow_ball_y_action = FollowBallVerticalDefender("Assistant follow ball y action", "/defender", field_data, team_command, x_position=DEFENDER_LINE)
    assistant_follow_ball_y_node = py_trees.composites.Sequence("Assistant follow ball y node", children=[assistant_go_back, assistant_follow_ball_y_action])
    ### Assistant follow ball y node

    ball_in_defense_actions = py_trees.composites.Selector("Ball in defense actions", children=[ball_near_goal_node, get_ball_node, assistant_follow_ball_y_node])
    ## Ball in defense actions
    ball_in_defense_node = py_trees.composites.Parallel("Ball in defense node", children=[ball_in_defense_condition, ball_in_defense_actions])
    # Ball in defense node

    ''' Ball not in defense '''
    # Ball in attack node
    stay_behind_node = GoBehindBall("Ball in attack node", "/defender", field_data, team_command, distance = -DEFENDER_LINE)
    look_at_ball_node = LookAtBall("Look at Ball", "/defender", field_data, team_command)
    ball_in_attack_node = py_trees.composites.Sequence(name="Attack node", children=[stay_behind_node, look_at_ball_node])
    # Ball in attack node

    # Add children nodes
    root.add_child(ball_in_defense_node)
    root.add_child(ball_in_attack_node)

    return root


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
    get_ball_condition_two = xBallLTd("Get Ball Condition 2", "/striker", field_data, DEFENDER_AREA_X)
    get_ball_inverter = py_trees.decorators.Inverter(name="Get Ball Inverter", child=get_ball_condition_two)
    get_ball_selector = py_trees.composites.Selector(name="Get Ball Selector", children=[
        get_ball_condition_one, get_ball_inverter
    ])
    get_ball_action = GetBall("Get Ball Action", "/striker", field_data, team_command, DEFENDER_AREA_X)
    get_ball_parallel = py_trees.composites.Parallel(name="Get Ball Node", children=[get_ball_selector, get_ball_action])
    root.add_child(get_ball_parallel)

    wait_to_strike_node = WaitToStrike("Wait to Strike Node", "/striker", field_data, team_command, DEFENDER_AREA_X)
    root.add_child(wait_to_strike_node)

    return root
