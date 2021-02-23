import numpy as np

def versor(vector):
    """ Returns the unit vector of the vector."""
    if np.linalg.norm(vector) == 0:
        return np.array([0, 0])
    return vector / np.linalg.norm(vector)

def from_polar(angle, magnitude = 1.0):
    return np.array([np.cos(angle), np.sin(angle)]) * magnitude

def rotate(vector, radians):
    """Use numpy to build a rotation matrix and take the dot product."""
    vector = np.array(vector)
    c, s = np.cos(radians), np.sin(radians)
    R = np.array([[c, -s], [s, c]])
    return np.dot(R, vector)

def assert_angle(angle):
    angle = angle % (2 * np.pi)

    if angle > np.pi:
        angle -= 2 * np.pi

    return angle

def gaussian(x, std_dev = 1.0, mean = 0.0, height = 1.0):
    return height * np.exp(-np.power(x - mean, 2.) / (2 * np.power(std_dev, 2.)))

def vectors_angle(vector1, vector2=np.array((1,0))):
    """Use numpy to calculate the angle between 2 vectors"""
    dot_product = np.dot(versor(vector1), versor(vector2))
    angle = np.arccos(dot_product)

    return angle
