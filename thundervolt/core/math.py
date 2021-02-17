import numpy as np

def versor(vector):
    """ Returns the unit vector of the vector."""
    if np.linalg.norm(vector) == 0:
        return np.array([0, 0])
    return vector / np.linalg.norm(vector)

def rotate(vector, radians):
    """Use numpy to build a rotation matrix and take the dot product."""
    c, s = np.cos(radians), np.sin(radians)
    R = np.array([[c, -s], [s, c]])
    return np.dot(R, vector.T)
