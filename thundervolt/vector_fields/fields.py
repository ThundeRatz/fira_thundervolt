import numpy as np
import random
import json

from thundervolt.core import math, data

def call_or_return(func, field_data):
    if callable(func):
        return func(field_data)
    return func

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
    def __init__(self, field_data, **kwargs):
        self.field_data = field_data
        self.name = kwargs.get('name', '{}|{}'.format(self.__class__, random.random() * 10000))
        self.output = None
        self.field_childrens = []


    def add(self, field):
        self.field_childrens.append(field)


    def compute(self, pose: data.Pose2D):
        output_sum = np.zeros(2)

        for field in self.field_childrens:
            output = field.compute(pose)
            output_sum += output

        self.output = output_sum

        return output_sum

class RadialField(VectorField):
    def __init__(self, field_data, **kwargs):
        super().__init__(field_data, **kwargs)

        # Point definition
        self.target = kwargs.get('target')
        self.repelling = kwargs.get('repelling', False)

        # Geometric configuration
        self.max_radius = kwargs.get('max_radius', None)
        self.decay_radius = kwargs.get('decay_radius', None)
        self.field_limits = kwargs.get('field_limits', None)

        # Weight
        self.multiplier = kwargs.get('multiplier', 1)

    def compute(self, pose: data.Pose2D):
        position = np.array([pose.x, pose.y])
        field_limits = call_or_return(self.field_limits, self.field_data)

        if field_limits and not(-field_limits[0] <= position[0] <= field_limits[0]):
            return np.zeros(2)

        if field_limits and not(-field_limits[1] <= position[1] <= field_limits[1]):
            return np.zeros(2)

        target = np.array(call_or_return(self.target, self.field_data))
        max_radius = call_or_return(self.max_radius, self.field_data)

        to_target = np.subtract(target, position)
        to_taget_scalar = np.linalg.norm(to_target)
        to_target = math.versor(to_target)

        if max_radius and to_taget_scalar > max_radius:
            return np.zeros(2)

        decay_radius = call_or_return(self.decay_radius, self.field_data)
        repelling = call_or_return(self.repelling, self.field_data)
        multiplier = call_or_return(self.multiplier, self.field_data)

        if repelling:
            multiplier *= -1

        decay = 1
        if max_radius and decay_radius:
            decay = max(0, min(1, (max_radius - to_taget_scalar)/(max_radius - decay_radius)))

        return to_target * decay * multiplier


class LineField(VectorField):
    def __init__(self, field_data, **kwargs):
        super().__init__(field_data, **kwargs)

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

        # Weight
        self.multiplier = kwargs.get('multiplier', 1)

    def compute(self, pose: data.Pose2D):
        position = np.array([pose.x, pose.y])

        field_limits = call_or_return(self.field_limits, self.field_data)

        if field_limits and not(-field_limits[0] <= position[0] <= field_limits[0]):
            return np.zeros(2)

        if field_limits and not(-field_limits[1] <= position[1] <= field_limits[1]):
            return np.zeros(2)

        target = np.array(call_or_return(self.target, self.field_data))
        theta = call_or_return(self.theta, self.field_data)
        max_dist = call_or_return(self.max_dist, self.field_data)
        size = call_or_return(self.size, self.field_data)
        only_forward = call_or_return(self.only_forward, self.field_data)
        side = call_or_return(self.side, self.field_data)

        # Ortogonal Projections
        to_position = position - target
        line_dir = math.from_polar(theta)
        axis_dir = math.from_polar(theta - np.pi / 2)

        proj_line = np.dot(to_position, line_dir)
        proj_axis = np.dot(to_position, axis_dir)

        # Check axis direction conditions
        if side == 'positive' and proj_axis < 0:
            return np.zeros(2)

        if side == 'negative' and proj_axis > 0:
            return np.zeros(2)

        if max_dist and abs(proj_axis) > max_dist:
            return np.zeros(2)

        # Check line direction conditions
        if only_forward and proj_line < 0:
            return np.zeros(2)

        if size and abs(proj_line) > size:
            return np.zeros(2)

        # Define direction
        output = axis_dir
        if proj_axis > 0:
            output *= -1

        repelling = call_or_return(self.repelling, self.field_data)
        multiplier = call_or_return(self.multiplier, self.field_data)

        if self.repelling:
            multiplier *= -1

        # Calculate decay
        decay_dist = call_or_return(self.decay_dist, self.field_data)

        decay = 1
        if max_dist and decay_dist:
            decay = max(0, min(1, abs((max_dist - abs(proj_axis))/(max_dist - decay_dist))))

        return output * decay * multiplier

class TangentField(VectorField):
    def __init__(self, field_data, **kwargs):
        super().__init__(field_data, **kwargs)

        # Hyperbolic spiral definition
        self.target = kwargs.get('target')
        self.radius = kwargs.get('radius')
        self.clockwise = kwargs.get('clockwise', False)
        self.K = kwargs.get('K', 1/2500)

        # Geometric configuration
        self.max_radius = kwargs.get('max_radius', None)
        self.decay_radius = kwargs.get('decay_radius', None)
        self.field_limits = kwargs.get('field_limits', None)

        # Weight
        self.multiplier = kwargs.get('multiplier', 1)

    def compute(self, pose: data.Pose2D):
        position = np.array([pose.x, pose.y])

        field_limits = call_or_return(self.field_limits, self.field_data)

        if field_limits and not(-field_limits[0] <= position[0] <= field_limits[0]):
            return np.zeros(2)

        if field_limits and not(-field_limits[1] <= position[1] <= field_limits[1]):
            return np.zeros(2)

        target = np.array(call_or_return(self.target, self.field_data))
        max_radius = call_or_return(self.max_radius, self.field_data)

        to_position = position - target
        to_position_scalar = np.linalg.norm(to_position)

        if max_radius and to_position_scalar > max_radius:
            return np.zeros(2)

        multiplier = call_or_return(self.multiplier, self.field_data)
        decay_radius = call_or_return(self.decay_radius, self.field_data)
        radius = call_or_return(self.radius, self.field_data)
        rotation_dir = -1 if call_or_return(self.clockwise, self.field_data) else 1
        angle_to_position = np.arctan2(to_position[1], to_position[0])

        if to_position_scalar > self.radius:
            end_angle = angle_to_position + rotation_dir * (np.pi/2) * (2 - ((self.radius + self.K)/(to_position_scalar + self.K)))
        else:
            end_angle = angle_to_position + rotation_dir * (np.pi/2) * np.sqrt(to_position_scalar/self.radius)

        output = math.from_polar(end_angle)

        decay = 1
        if max_radius and decay_radius:
            decay = max(0, min(1, (max_radius - to_position_scalar)/(max_radius - decay_radius)))

        return output * decay * multiplier
