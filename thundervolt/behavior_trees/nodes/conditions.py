import py_trees

from .execution_node import ExecutionNode
from thundervolt.core import utils, data

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
    def __init__(self, name, role, field_data, player_tolerance, goal_tolerance):
        super().__init__(name, role, field_data)

        self.player_tol = player_tolerance
        self.goal_tol = goal_tolerance
        self.lpost = np.array((data.FIELD_LENGTH/2, -data.GOAL_WIDTH/2))
        self.rpost = np.array((data.FIELD_LENGTH/2, data.GOAL_WIDTH/2))

    def update(self):
        player_pos = np.zeros(2)
        player_pos[0] = self.field_data.robots[self.parameters.robot_id].position.x
        player_pos[1] = self.field_data.robots[self.parameters.robot_id].position.y
        player_theta = assert_half_angle(self.field_data.robots[self.parameters.robot_id].theta)
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))

        player_ball = np.array((ball_pos[0] - player_pos[0], ball_pos[1] - player_pos[1]))
        player_lpost = np.array((self.lpost[0] - player_pos[0], self.lpost[1] - player_pos[1]))
        player_rpost = np.array((self.rpost[0] - player_pos[0], self.rpost[1] - player_pos[1]))
        
        angle_ball = utils.vectors_angle(player_ball)
        angle_lpost = utils.vectors_angle(player_lpost)
        angle_rpost = utils.vectors_angle(player_rpost)

        if ((player_theta-self.player_tol <= angle_ball <= player_theta+self.player_tol)
        and (angle_lpost-self.goal_tol <= angle_ball <= angle_rpost+self.goal_tol)):
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
