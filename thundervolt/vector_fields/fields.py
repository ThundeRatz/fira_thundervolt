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
        self.repelling = kwargs.get('repelling', False)

        # Geometric configuration
        self.decay_dist = kwargs.get('decay_dist', None)
        self.max_dist = kwargs.get('max_dist', None)
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

        to_target = target - position
        line_dir = math.from_polar(theta)
        axis_dir = math.from_polar(theta + np.pi / 2)

        dist_to_line = math.distance_to_line(position, target, axis_dir)
        dist_to_axis = math.distance_to_line(position, target, line_dir)

        if self.size and dist_to_axis > self.size:
            return np.zeros(2)

        if max_dist and dist_to_line > max_dist:
            return np.zeros(2)

        output = axis_dir
        if np.dot(axis_dir, to_target) < 0:
            output *= -1

        decay_dist = call_or_return(self.decay_dist, self.field_data)
        repelling = call_or_return(self.repelling, self.field_data)
        multiplier = call_or_return(self.multiplier, self.field_data)

        if self.repelling:
            multiplier *= -1

        decay = 1
        if max_dist and decay_dist:
            decay = max(0, min(1, abs((max_dist - dist_to_line)/(max_dist - decay_dist))))

        return output * decay * multiplier

class TangentialField(PotentialField):
    def __init__(self, field_data, **kwargs):
        super().__init__(field_data, **kwargs)
        self.target = kwargs['target']
        self.clockwise = kwargs.get('clockwise', False)
        self.decay = kwargs['decay']
        self.radius = kwargs.get('radius', kwargs.get('radius_max'))
        self.radius_max = kwargs.get('radius_max')
        self.multiplier = kwargs.get('multiplier', 1)
        self.orbitation_speed = kwargs.get('orbitation_speed', self.multiplier)

        self.K = kwargs.get('K', 1/25000)

        self.field_limits = kwargs.get('field_limits', None)

    def compute(self, input):
        target_go_to = call_or_return(self.target, self.field_data)
        radius_max = call_or_return(self.radius_max, self.field_data)
        multiplier = call_or_return(self.multiplier, self.field_data)

        cwo = 1 if call_or_return(self.clockwise, self.field_data) else -1 # clockwise ou counterclockwise

        to_target = np.subtract(target_go_to, input)
        to_taget_scalar = np.linalg.norm(to_target)

        angle_to_target = np.arctan2(target_go_to[1] - input[1], target_go_to[0] - input[0] )

        if self.field_limits and not(0 <= input[0] <= self.field_limits[0]):
            return (0, 0)

        if self.field_limits and not(0 <= input[1] <= self.field_limits[1]):
            return (0, 0)

        if radius_max and to_taget_scalar > radius_max:
            return (0, 0)

        to_target_scalar_norm = max(0, min(1, abs((self.radius - to_taget_scalar)/radius_max)))
        end_angle = 0
        if to_taget_scalar > self.radius:
            end_angle = angle_to_target + cwo * (np.pi/2) * (2 - ( (self.radius + self.K)/(to_taget_scalar + self.K) ))
        else:
            end_angle = angle_to_target + cwo * (np.pi/2) * np.sqrt(to_taget_scalar/self.radius)

        to_target_norm = -math.versor( (np.cos(end_angle), np.sin(end_angle)) )

        force = apply_decay(self.decay, to_target_scalar_norm)

        return (
            to_target_norm[0] * force * multiplier,
            to_target_norm[1] * force * multiplier
        )
