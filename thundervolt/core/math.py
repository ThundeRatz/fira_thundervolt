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
    return np.dot(R, vector.T)
