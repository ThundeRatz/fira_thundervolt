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

# Defender imports - para controle
from thundervolt.behavior_trees.nodes.action.clear_ball import ClearBall #repetido
from thundervolt.behavior_trees.nodes.conditions import BallDistToPlayerLTd #repetido
from thundervolt.behavior_trees.nodes.action.follow_ball_x import FollowBallHorizontal
from thundervolt.behavior_trees.nodes.action.defend_corner import DefendCorner
from thundervolt.behavior_trees.nodes.action.go_near_corner import GoNearCorner
from thundervolt.behavior_trees.nodes.conditions import yBallLTd #Temporario
from thundervolt.behavior_trees.nodes.conditions import xPlayerLTd
from thundervolt.behavior_trees.nodes.conditions import xPlayerLTxBall
from thundervolt.behavior_trees.nodes.action.go_behind_ball import GoBehindBall
from thundervolt.behavior_trees.nodes.conditions import xBallLTd
from thundervolt.behavior_trees.nodes.action.get_ball import GetBall
from thundervolt.behavior_trees.nodes.action.follow_ball_y import FollowBallVertical
from thundervolt.behavior_trees.nodes.conditions import FoeCloserToBall

PLAYER_DIST_TO_GOAL = 1.5 * data.ROBOT_SIZE
BALL_DIST_TO_PLAYER = 0.75 * data.ROBOT_SIZE + data.BALL_RADIUS
BALL_DIST_TO_GOAL = 1.5 * data.ROBOT_SIZE + data.BALL_RADIUS
AREA_LINE_X = data.FIELD_LENGTH/2 - data.GOAL_AREA_DEPTH
CLOSE_LINE_Y = data.GOAL_WIDTH/2 + data.ROBOT_SIZE
LINE_X = data.FIELD_LENGTH/2 - data.ROBOT_SIZE

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

def create_defender_tree(field_data, team_command):
    root = py_trees.composites.Selector("Root")

    ''' Defender-defender mode '''
    # Defender-defender mode node
    foe_closer_to_ball_condition = FoeCloserToBall("Foe closer to ball condition", "/defender", field_data)
    ## Actions opponent closer to ball node --> TESTANDO
    ### Ball in defense node --> TESTANDO
    ball_in_defense_condition = xBallLTd("Ball in defense condition", "/defender", field_data, 0.0)
    #### Actions ball in defense node --> PARECE OK!
    ##### Ball near goal node --> PARECE OK!
    ball_near_goal_condition = xBallLTd("Distance (ball, goal) less than d condition", "/defender", field_data, -0.45)

    ###### Corner node -> ATÉ AQUI FUNCIONA +-
    go_near_corner_action = GoNearCorner("Go near corner action", "/defender", field_data, team_command, y_position = 0.45)
    ####### After go to corner node
    ######## Spin node
    spin_condition = BallDistToPlayerLTd("Spin Condition", "/defender", field_data, BALL_DIST_TO_PLAYER)
    spin_action = ClearBall("Clear Ball Action", "/defender", field_data, team_command)
    spin_node = py_trees.composites.Parallel(name="Spin Node", children=[spin_condition, spin_action])
    ####### Spin node #######
    ######## Defend corner node
    defend_corner_action = DefendCorner("Defend corner action", "/defender", field_data, team_command, y_position = 0.45)
    ######## Defend corner node ########
    after_go_corner_node = py_trees.composites.Selector("After go to corner node", [spin_node, defend_corner_action])
    ####### After go to corner node #######
    corner_node = py_trees.composites.Sequence(name="Corner node", children=[go_near_corner_action, after_go_corner_node])
    ######Corner node ###### -> ATÉ AQUI FUNCIONA +-

    ball_near_goal_node = py_trees.composites.Parallel(name="Ball near goal node", children=[ball_near_goal_condition, corner_node])
    ##### Ball near goal node ##### --> PARECE OK!
    get_ball_action = GetBall("Get ball action", "/defender", field_data, team_command)
    actions_ball_in_defense_node = py_trees.composites.Selector(name="Actions ball in defense node", children=[ball_near_goal_node, get_ball_action])
    #### Actions ball in defense node --> PARECE OK!
    ball_in_defense_node = py_trees.composites.Parallel(name="Ball in defense node", children=[ball_in_defense_condition, actions_ball_in_defense_node])
    ### Ball in defense node --> TESTANDO
    ### Defender go back node -> FUnciona, mas ainda precisa calibrar melhor os parametros do GoBehindBall
    #### Go back conditions
    go_back_condition1 = xPlayerLTd("GO back condition 1", "/defender", field_data, data.FIELD_LENGTH/4)
    go_back_inverter1 = py_trees.decorators.Inverter(name="GO back condition 1 Inverter", child=go_back_condition1)
    go_back_condition2 = xPlayerLTxBall("GO back condition 1", "/defender", field_data)
    go_back_inverter2 = py_trees.decorators.Inverter(name="GO back condition 2 Inverter", child=go_back_condition2)
    go_back_conditions_node = py_trees.composites.Selector(name="Go back conditions node", children=[go_back_inverter1, go_back_inverter2])
    #### Go back conditions ####
    #### Go back to defense action
    go_back_to_defense_action = GoBehindBall("Defendr Go back to defense action", "/defender", field_data, team_command, 0.4)
    #### Go back to defense action
    defender_go_back_node = py_trees.composites.Parallel("Defender go back node", children=[go_back_conditions_node, go_back_to_defense_action])
    ### Defender go back node ### --> Parece ok
    ### Defender follow ball y action --> Parece ok
    defender_follow_ball_y_action = FollowBallVertical("Defender follow ball y action", "/defender", field_data, team_command, x_position=0.0)
    ### Defender follow ball y action ### --> Parece ok
    actions_oponnent_closer_to_ball_node = py_trees.composites.Selector("Actions opponent closer to ball", children=[ball_in_defense_node, defender_go_back_node, defender_follow_ball_y_action])
    ## Actions opponent closer to ball node ##
    defender_defender_mode_node = py_trees.composites.Parallel("Defender-defender mode node", children=[foe_closer_to_ball_condition, actions_oponnent_closer_to_ball_node])
    # Defender-defender mode node

    ''' Defender-assistant mode '''
    # Defender-assistant mode node -->Testando
    ## Assistant follow ball y node
    assistant_follow_ball_y_condition = xPlayerLTxBall("Assistant go back condition ", "/defender", field_data)
    assistant_follow_ball_y_action = FollowBallVertical("Assistant follow ball y action", "/defender", field_data, team_command, x_position=-data.FIELD_LENGTH/4)
    assistant_follow_ball_y_node = py_trees.composites.Parallel("Assistant follow ball y node", children=[assistant_follow_ball_y_condition, assistant_follow_ball_y_action])
    ## Follow ball y 2 node
    ## Assistant go back action
    assistant_go_back_action = GoBehindBall("Assistant go back to defense action", "/defender", field_data, team_command, -data.FIELD_LENGTH/4)
    ## Assistant go back action
    defender_assistant_mode_node = py_trees.composites.Selector("Defender-assistant mode node", children=[assistant_follow_ball_y_node, assistant_go_back_action])
    # Defender-assistant mode node --> Falta o caso em que a bola está próximo da área (o assistente nao poderia ir para tras da bola por causa do limite do go behind ball e entao ficaria seguindo a bola, atrapalhandoa jogada)

    # Add children nodes
    root.add_child(defender_defender_mode_node)
    root.add_child(defender_assistant_mode_node)

    return root

def CreateStrikerTree():
    pass
