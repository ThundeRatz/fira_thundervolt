import pytest
import py_trees

from thundervolt.core.data import FieldData
from thundervolt.behavior_trees.nodes.execution_node import ExecutionNode

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
