import numpy as np
import random
import json

from thundervolt.core import math, data

class PotentialDataExporter(object):
    def __init__(self, name):
        self.file = open(
            '{}|potential_field.log'.format(name),
            'w'
        )

    def export(self, field, robot, ball):
        X = []
        Y = []
        U = []
        V = []

        for x in range(-10, 150 + 10, 2):
            x = x/100.0
            for y in range(-10, 130 + 10, 2):
                y = y/100.0
                res = field.compute([x, y])
                X.append(x)
                Y.append(y)
                U.append(res[0])
                V.append(res[1])

        plot_file = {
            "x": X,
            "y": Y,
            "u": U,
            "v": V,
            "robot_x": robot.x,
            "robot_y": robot.y,
            "ball_x": ball.x,
            "ball_y": ball.y,
            "field": field.name
        }

        self.file.write(json.dumps(plot_file) + "||")

class VectorField(object):
    def __init__(self, **kwargs):
        self.name = kwargs.get('name', '{}|{}'.format(self.__class__, random.random() * 10000))
        self.output = None
        self.field_childrens = []

    def add(self, field):
        self.field_childrens.append(field)

    def update(self, field_data, robot_id):
        for field in self.field_childrens:
            field.update(field_data, robot_id)

    def compute(self, pose):
        output_sum = np.zeros(2)

        for field in self.field_childrens:
            output = field.compute(pose)
            output_sum += output

        self.output = output_sum

        return output_sum

class RadialField(VectorField):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Point definition
        self.target = kwargs.get('target')
        self.repelling = kwargs.get('repelling', False)

        # Geometric configuration
        self.max_radius = kwargs.get('max_radius', None)
        self.decay_radius = kwargs.get('decay_radius', None)
        self.field_limits = kwargs.get('field_limits', None)

        # General
        self.multiplier = kwargs.get('multiplier', 1)
        self.update_rule = kwargs.get('update_rule', None)

    def update(self, field_data, robot_id):
        if callable(self.update_rule):
            self.update_rule(self, field_data, robot_id)

    def compute(self, pose):
        position = np.array(pose)

        if self.field_limits and not(-self.field_limits[0] <= position[0] <= self.field_limits[0]):
            return np.zeros(2)

        if self.field_limits and not(-self.field_limits[1] <= position[1] <= self.field_limits[1]):
            return np.zeros(2)

        to_target = np.subtract(np.array(self.target), position)
        to_taget_scalar = np.linalg.norm(to_target)
        to_target = math.versor(to_target)

        if self.max_radius and to_taget_scalar > self.max_radius:
            return np.zeros(2)

        # Define direction
        output = to_target
        if self.repelling:
            output *= -1

        decay = 1
        if self.max_radius and self.decay_radius:
            decay = max(0, min(1, (self.max_radius - to_taget_scalar)/(self.max_radius - self.decay_radius)))

        return output * decay * self.multiplier


class LineField(VectorField):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Line definition
        self.target = kwargs.get('target')
        self.theta = kwargs.get('theta')
        self.size = kwargs.get('size')
        self.only_forward = kwargs.get('only_forward', False)
        self.side = kwargs.get('side', 'both')
        self.repelling = kwargs.get('repelling', False)

        # Geometric configuration
        self.max_dist = kwargs.get('max_dist', None)
        self.decay_dist = kwargs.get('decay_dist', None)
        self.field_limits = kwargs.get('field_limits', None)

        # General
        self.multiplier = kwargs.get('multiplier', 1)
        self.update_rule = kwargs.get('update_rule', None)

    def update(self, field_data, robot_id):
        if callable(self.update_rule):
            self.update_rule(self, field_data, robot_id)

    def compute(self, pose):
        position = np.array(pose)

        if self.field_limits and not(-self.field_limits[0] <= position[0] <= self.field_limits[0]):
            return np.zeros(2)

        if self.field_limits and not(-self.field_limits[1] <= position[1] <= self.field_limits[1]):
            return np.zeros(2)

        # Ortogonal Projections
        to_position = position - np.array(self.target)
        line_dir = math.from_polar(self.theta)
        axis_dir = math.from_polar(self.theta - np.pi / 2)

        proj_line = np.dot(to_position, line_dir)
        proj_axis = np.dot(to_position, axis_dir)

        # Check axis direction conditions
        if self.side == 'positive' and proj_axis < 0:
            return np.zeros(2)

        if self.side == 'negative' and proj_axis > 0:
            return np.zeros(2)

        if self.max_dist and abs(proj_axis) > self.max_dist:
            return np.zeros(2)

        # Check line direction conditions
        if self.only_forward and proj_line < 0:
            return np.zeros(2)

        if self.size and abs(proj_line) > self.size:
            return np.zeros(2)

        # Define direction
        output = axis_dir
        if proj_axis > 0:
            output *= -1

        if self.repelling:
            output *= -1

        # Calculate decay
        decay = 1
        if self.max_dist and self.decay_dist:
            decay = max(0, min(1, abs((self.max_dist - abs(proj_axis))/(self.max_dist - self.decay_dist))))

        return output * decay * self.multiplier

class TangentField(VectorField):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Hyperbolic spiral definition
        self.target = kwargs.get('target')
        self.radius = kwargs.get('radius')
        self.clockwise = kwargs.get('clockwise', False)
        self.damping = kwargs.get('damping', 1/2500)

        # Geometric configuration
        self.max_radius = kwargs.get('max_radius', None)
        self.decay_radius = kwargs.get('decay_radius', None)
        self.field_limits = kwargs.get('field_limits', None)

        # General
        self.multiplier = kwargs.get('multiplier', 1)
        self.update_rule = kwargs.get('update_rule', None)

    def update(self, field_data, robot_id):
        if callable(self.update_rule):
            self.update_rule(self, field_data, robot_id)

    def compute(self, pose):
        position = np.array(pose)

        if self.field_limits and not(-self.field_limits[0] <= position[0] <= self.field_limits[0]):
            return np.zeros(2)

        if self.field_limits and not(-self.field_limits[1] <= position[1] <= self.field_limits[1]):
            return np.zeros(2)

        to_position = position - np.array(self.target)
        to_position_scalar = np.linalg.norm(to_position)

        if self.max_radius and to_position_scalar > self.max_radius:
            return np.zeros(2)

        rotation_dir = -1 if self.clockwise else 1
        angle_to_position = np.arctan2(to_position[1], to_position[0])

        if to_position_scalar > self.radius:
            end_angle = angle_to_position + rotation_dir * (np.pi/2) * (2 - ((self.radius + self.damping) / (to_position_scalar + self.damping)))
        else:
            end_angle = angle_to_position + rotation_dir * (np.pi/2) * np.sqrt(to_position_scalar / self.radius)

        output = math.from_polar(end_angle)

        decay = 1
        if self.max_radius and self.decay_radius:
            decay = max(0, min(1, (self.max_radius - to_position_scalar)/(self.max_radius - self.decay_radius)))

        return output * decay * self.multiplier
