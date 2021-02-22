import pytest
import py_trees

from thundervolt.core.data import FieldData
from thundervolt.behavior_trees.nodes.execution_node import ExecutionNode
from thundervolt.behavior_trees.nodes.conditions import xPlayerLTd

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

