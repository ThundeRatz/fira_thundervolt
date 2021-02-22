import py_trees

from .execution_node import ExecutionNode

class xPlayerLTd(ExecutionNode):
    def __init__(self, name, role, field_data, d_position):
        super().__init__(name, role, field_data)

        self.d_position = d_position

    def update(self):
        if self.field_data.robots[self.parameters.robot_id].position.x < self.d_position:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

