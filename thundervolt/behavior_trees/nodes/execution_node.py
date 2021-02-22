import py_trees
from py_trees.behaviour import Behaviour

class ExecutionNode(Behaviour):
    def __init__(self, name, role):
        """
        Initialize execution node

        Args:
            name (str): Behaviour name
            role (str): Robot role namespace for black board client
        """
        super(ExecutionNode, self).__init__(name)

        self.parameters = self.attach_blackboard_client(name="Role selector", namespace=role)
        self.parameters.register_key('robot_id', access=py_trees.common.Access.READ)
