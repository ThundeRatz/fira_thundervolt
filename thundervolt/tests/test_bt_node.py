import pytest
import py_trees

from thundervolt.core.data import FieldData
from thundervolt.behavior_trees.nodes.execution_node import ExecutionNode
from thundervolt.behavior_trees.nodes.conditions import xPlayerLTd
from thundervolt.behavior_trees.nodes.conditions import GoodStrikerOrientation
from thundervolt.behavior_trees.nodes.conditions import xPlayerLTxBall
from thundervolt.behavior_trees.nodes.conditions import xBallLTd
from thundervolt.behavior_trees.nodes.conditions import BallDistToGoalLTd
from thundervolt.behavior_trees.nodes.conditions import FoeCloseToBall
from thundervolt.behavior_trees.nodes.conditions import GoalKeeperOutsideGoal
from thundervolt.behavior_trees.nodes.conditions import BallDistToPlayerLTd
from thundervolt.behavior_trees.nodes.conditions import TimeBallReachGoalLTt

def test_execution_node():
    class ExampleNode(ExecutionNode):
        def __init__(self, name, role, field_data):
            super().__init__(name, role, field_data)

        def setup(self):
            pass

        def initialise(self):
            pass

        def update(self):
            if self.parameters.robot_id == 1:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        def terminate(self, new_status):
            pass

    field_data = FieldData()
    my_node = ExampleNode("First Node", "/defender", field_data)
    bb_client = py_trees.blackboard.Client()
    bb_client.register_key(key="/defender/robot_id", access=py_trees.common.Access.WRITE)

    bb_client.defender.robot_id = 2
    my_node.tick_once()
    assert my_node.status is py_trees.common.Status.FAILURE

    bb_client.defender.robot_id = 1
    my_node.tick_once()
    assert my_node.status is py_trees.common.Status.SUCCESS


def test_x_player_lt_d():
    d_positions = [-0.1, -0.7, 0, 0.3, 0.5]
    player_x_postions = [-0.1, 0.1, -0.1, -0.45, 0]
    desired_status = [
        py_trees.common.Status.FAILURE,
        py_trees.common.Status.FAILURE,
        py_trees.common.Status.SUCCESS,
        py_trees.common.Status.SUCCESS,
        py_trees.common.Status.SUCCESS
    ]

    ROBOT_ID = 0

    field_data = FieldData()
    bb_client = py_trees.blackboard.Client()
    bb_client.register_key(key="/striker/robot_id", access=py_trees.common.Access.WRITE)
    bb_client.striker.robot_id = ROBOT_ID

    for i in range(len(d_positions)):
        print(f"TEST NUMBER: {i}")

        field_data.robots[ROBOT_ID].position.x = player_x_postions[i]

        cond_node = xPlayerLTd(f"X player less than {d_positions[i]}", "/striker", field_data, d_positions[i])
        cond_node.tick_once()

        assert cond_node.status is desired_status[i]


def test_good_striker_orientation():
    pass


def test_x_player_lt_x_ball():
    ball_x_positions = [-0.5, -0.3, -0.1, 0, 0, 0.1, 0.3, 0.5]
    player_x_postions = [0, -0.5, -0.4, 0.4, -0.2, 0.5, 0.5, -0.3]
    desired_status = [
        py_trees.common.Status.FAILURE,
        py_trees.common.Status.SUCCESS,
        py_trees.common.Status.SUCCESS,
        py_trees.common.Status.FAILURE,
        py_trees.common.Status.SUCCESS,
        py_trees.common.Status.FAILURE,
        py_trees.common.Status.FAILURE,
        py_trees.common.Status.SUCCESS
    ]

    ROBOT_ID = 0

    field_data = FieldData()
    bb_client = py_trees.blackboard.Client()
    bb_client.register_key(key="/striker/robot_id", access=py_trees.common.Access.WRITE)
    bb_client.striker.robot_id = ROBOT_ID

    for i in range(len(ball_x_positions)):
        print(f"TEST NUMBER: {i}")

        field_data.robots[ROBOT_ID].position.x = player_x_postions[i]

        cond_node = xPlayerLTxBall(f"X player less than {player_x_postions[i]}", "/striker", field_data, player_x_postions[i])
        cond_node.tick_once()

        assert cond_node.status is desired_status[i]


def test_x_ball_lt_d():
    pass


def test_ball_dist_to_goal_lt_d():
    pass


def test_foe_close_to_ball():
    pass


def test_goal_keeper_outside_goal():
    pass


def test_ball_dist_to_player_lt_d():
    pass


def test_time_ball_reach_goal_lt_t():
    pass

