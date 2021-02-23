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


class GoodStrikerOrientation(ExecutionNode):
    def __init__(self, name, role, field_data):
        super().__init__(name, role, field_data)

    def update(self):
        player_pos = np.zeros(2)
        player_pos[0] = self.field_data.robots[self.parameters.robot_id].position.x
        player_pos[1] = self.field_data.robots[self.parameters.robot_id].position.y
        player_theta = self.field_data.robots[self.parameters.robot_id].theta
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))

        player_ball = np.array((ball_pos[0] - player_pos[0], ball_pos[1] - player_pos[1]))
        angle = np.math.atan2(np.linalg.det([np.array((1,0)), player_ball]),np.dot(np.array((1,0)),player_ball))

        if player_theta-1 <= angle <= player_theta+1:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class xPlayerLTxBall(ExecutionNode):
    pass


class xBallLTd(ExecutionNode):
    pass


class BallDistToGoalLTd(ExecutionNode):
    pass


class FoeCloseToBall(ExecutionNode):
    pass


class GoalKeeperOutsideGoal(ExecutionNode):
    pass


class BallDistToPlayerLTd(ExecutionNode):
    pass


class TimeBallReachGoalLTt(ExecutionNode):
    pass
