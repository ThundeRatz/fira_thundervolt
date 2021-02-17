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

        if self.field_limits and not(-self.field_limits[0] <= position[0] <= self.field_limits[0]):
            return np.zeros(2)

        if self.field_limits and not(-self.field_limits[1] <= position[1] <= self.field_limits[1]):
            return np.zeros(2)

        target_go_to = np.array(call_or_return(self.target, self.field_data))
        max_radius = call_or_return(self.max_radius, self.field_data)
        multiplier = call_or_return(self.multiplier, self.field_data)

        to_target = np.subtract(target_go_to, position)
        to_taget_scalar = np.linalg.norm(to_target)
        to_target = math.versor(to_target)

        if max_radius and to_taget_scalar > max_radius:
            return np.zeros(2)

        if self.repelling:
            multiplier *= -1

        decay = 1
        if self.max_radius and self.decay_radius:
            decay = max(0, min(1, (self.max_radius - to_taget_scalar)/(self.max_radius - self.decay_radius)))

        return to_target * decay * multiplier


class LineField(PotentialField):
    def __init__(self, field_data, **kwargs):
        super().__init__(field_data, **kwargs)
        self.target = kwargs['target']
        self.decay = kwargs['decay']

        self.multiplier = kwargs.get('multiplier', 1)

        # line definition
        self.theta = kwargs['theta']
        self.line_size = kwargs['line_size']
        self.line_dist = kwargs['line_dist']
        self.line_dist_max = kwargs.get('line_dist_max')

        self.line_size_single_side = kwargs.get('line_size_single_side', False)
        self.line_dist_single_side = kwargs.get('line_dist_single_side', False)

        self.inverse = kwargs.get('inverse', False)

        self.field_limits = kwargs.get('field_limits', None)

    def compute(self, input):
        target_line = call_or_return(self.target, self.field_data)
        target_theta = call_or_return(self.theta, self.field_data)

        multiplier = call_or_return(self.multiplier, self.field_data)
        line_dist_max = call_or_return(self.line_dist_max, self.field_data)

        to_line = np.subtract(target_line, input)
        to_line_with_theta = math.rotate(to_line, -target_theta)

        if self.field_limits and not(0 <= input[0] <= self.field_limits[0]):
            return (0, 0)

        if self.field_limits and not(0 <= input[1] <= self.field_limits[1]):
            return (0, 0)

        if self.line_size and abs(to_line_with_theta[0]) > self.line_size:
            return (0, 0)

        if self.line_size_single_side and to_line_with_theta[0] < 0:
            return (0, 0)

        if self.line_dist_max and abs(to_line_with_theta[1]) > line_dist_max:
            return (0, 0)

        if self.line_dist_single_side and to_line_with_theta[1] < 0 and not self.inverse:
            return(0, 0)

        if self.line_dist_single_side and to_line_with_theta[1] > 0 and self.inverse:
            return(0, 0)

        to_line_norm = math.versor(
            math.rotate(
                np.array([0, to_line_with_theta[1]]),
                target_theta
            )
        )

        to_line_scalar_norm = max(0, min(1, abs(to_line_with_theta[1]/self.line_dist)))

        force = apply_decay(self.decay, to_line_scalar_norm)

        return (
            to_line_norm[0] * force * multiplier,
            to_line_norm[1] * force * multiplier
        )

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