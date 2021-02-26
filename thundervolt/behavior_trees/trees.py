import py_trees

from thundervolt.core import data
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.conditions import PlayerDistToGoalLTd
from thundervolt.behavior_trees.nodes.action.back_to_goal import BackToGoal
from thundervolt.behavior_trees.nodes.conditions import BallDistToPlayerLTd
from thundervolt.behavior_trees.nodes.conditions import BallDistToGoalLTd
from thundervolt.behavior_trees.nodes.conditions import xBallLTd
from thundervolt.behavior_trees.nodes.action.clear_ball import ClearBall
from thundervolt.behavior_trees.nodes.action.save import SaveGoal
from thundervolt.behavior_trees.nodes.action.follow_ball_y import FollowBallVertical

from thundervolt.behavior_trees.nodes.action.follow_ball_x import FollowBallHorizontal
from thundervolt.behavior_trees.nodes.action.defend_corner import DefendCorner
from thundervolt.behavior_trees.nodes.action.go_near_corner import GoNearCorner
from thundervolt.behavior_trees.nodes.conditions import xPlayerLTd
from thundervolt.behavior_trees.nodes.conditions import xPlayerLTxBall
from thundervolt.behavior_trees.nodes.action.go_behind_ball import GoBehindBall
from thundervolt.behavior_trees.nodes.conditions import xBallLTd
from thundervolt.behavior_trees.nodes.action.get_ball_defender import GetBallDefender
from thundervolt.behavior_trees.nodes.action.follow_ball_y_defender import FollowBallVerticalDefender
from thundervolt.behavior_trees.nodes.conditions import FoeCloserToBall
from thundervolt.behavior_trees.nodes.action.go_back_goal_area import BackToGoalArea
from thundervolt.behavior_trees.nodes.action.look_at_ball import LookAtBall

PLAYER_DIST_TO_GOAL = 2.2 * data.ROBOT_SIZE
BALL_DIST_TO_PLAYER = 0.75 * data.ROBOT_SIZE + data.BALL_RADIUS
BALL_DIST_TO_GOAL = 1.25 * data.ROBOT_SIZE + data.BALL_RADIUS
AREA_LINE_X = data.FIELD_LENGTH/2 - data.GOAL_AREA_DEPTH
CLOSE_LINE_Y = data.GOAL_AREA_WIDTH/2
LINE_X = data.FIELD_LENGTH/2 - 0.8 * data.ROBOT_SIZE
SPIN_LINE = LINE_X - data.ROBOT_SIZE/2

DEFENDER_LINE = -0.5
STRIKER_X_LIMIT = -0.2

def create_goalkeeper_tree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    back_condition = PlayerDistToGoalLTd("Back Condition", "/goalkeeper", field_data, PLAYER_DIST_TO_GOAL)
    back_inverter = py_trees.decorators.Inverter(name="Back Inverter", child=back_condition)
    back_action = BackToGoal("Back Action", "/goalkeeper", field_data, team_command)
    back_node = py_trees.composites.Parallel(name="Back Node", children=[back_inverter, back_action])
    root.add_child(back_node)

    save_node = SaveGoal("Save Node", "/goalkeeper", field_data, team_command, 3.0, LINE_X, CLOSE_LINE_Y)
    root.add_child(save_node)

    spin_condition1 = BallDistToPlayerLTd("Spin Condition 1", "/goalkeeper", field_data, BALL_DIST_TO_PLAYER)
    spin_condition2 = BallDistToGoalLTd("Spin Condition 2", "/goalkeeper", field_data, BALL_DIST_TO_GOAL)
    spin_condition3 = xBallLTd("Spin Condition 3", "/goalkeeper", field_data, -SPIN_LINE)
    spin_inverter = py_trees.decorators.Inverter(name="Spin Inverter", child=spin_condition2)
    clear_action = ClearBall("Clear Ball Action", "/goalkeeper", field_data, team_command)
    spin_node = py_trees.composites.Parallel(name="Spin Node", children=[spin_condition1, spin_inverter, spin_condition3, clear_action])
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

def create_defender_tree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    ''' Ball in defense '''
    # Ball in defense node
    ball_in_defense_condition = xBallLTd("Ball in defense condition", "/defender", field_data, 0.0)
    ## Ball in defense actions
    ### Ball near goal node
    ball_near_goal_condition = xBallLTd("Distance (ball, goal) less than d condition", "/defender", field_data, -data.FIELD_LENGTH/2 + data.GOAL_AREA_DEPTH)

    #### Corner node
    go_near_corner_action = GoNearCorner("Go near corner action", "/defender", field_data, team_command, y_position = 0.38)
    ##### After go to corner node
    ###### Spin node
    spin_condition = BallDistToPlayerLTd("Spin Condition", "/defender", field_data, BALL_DIST_TO_PLAYER)
    spin_action = ClearBall("Clear Ball Action", "/defender", field_data, team_command)
    spin_node = py_trees.composites.Parallel(name="Spin Node", children=[spin_condition, spin_action])
    ##### Spin node
    ###### Defend corner node
    defend_corner_action = DefendCorner("Defend corner action", "/defender", field_data, team_command, y_position = 0.45)
    ###### Defend corner node
    after_go_corner_node = py_trees.composites.Selector("After go to corner node", [spin_node, defend_corner_action])
    ##### After go to corner node
    corner_node = py_trees.composites.Sequence(name="Corner node", children=[go_near_corner_action, after_go_corner_node])
    #### Corner node
    ball_near_goal_node = py_trees.composites.Parallel(name="Ball near goal node", children=[ball_near_goal_condition, corner_node])
    ### Ball near goal node

    ### Foe closer node
    foe_closer_to_ball_condition = FoeCloserToBall("Foe closer to ball condition", "/defender", field_data)
    ball_x_lt_limit_striker_condition = xBallLTd("Ball x less than limit striker", "/defender", field_data, d_position = STRIKER_X_LIMIT)
    get_ball_action = GetBallDefender("Get ball action", "/defender", field_data, team_command)
    foe_closer_node = py_trees.composites.Parallel("Foe closer node", children=[ball_x_lt_limit_striker_condition, foe_closer_to_ball_condition, get_ball_action])
    ### Foe closer node

    ### Assistant follow ball y node
    assistant_follow_ball_y_condition = xPlayerLTxBall("Assistant go back condition ", "/defender", field_data)
    assistant_follow_ball_y_action = FollowBallVerticalDefender("Assistant follow ball y action", "/defender", field_data, team_command, x_position=DEFENDER_LINE)
    assistant_follow_ball_y_node = py_trees.composites.Parallel("Assistant follow ball y node", children=[assistant_follow_ball_y_condition, assistant_follow_ball_y_action])
    ### Assistant follow ball y node

    ### Assistant go back action
    assistant_go_back = BackToGoalArea("Go back to area line action", "/defender", field_data, team_command, -data.FIELD_LENGTH/3)
    ### Assistant go back action

    ball_in_defense_actions = py_trees.composites.Selector("Ball in defense actions", children=[ball_near_goal_node,foe_closer_node, assistant_follow_ball_y_node, assistant_go_back])
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

def CreateStrikerTree():
    pass
