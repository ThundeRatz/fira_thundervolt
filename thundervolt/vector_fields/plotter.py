import numpy as np
import matplotlib.pyplot as plt

from thundervolt.core import math, data
from thundervolt.vector_fields import fields


GAME_FIELD = np.array(
    [[-0.75, -0.75, -0.85, -0.85, -0.75, -0.75, 0.75, 0.75, 0.85,  0.85,  0.75,  0.75, -0.75],
     [-0.65, -0.20, -0.20,  0.20,  0.20,  0.65, 0.65, 0.20, 0.20, -0.20, -0.20, -0.65, -0.65]]
)
class FieldPlotter():
    def __init__(self, name, **kwargs):
        self.name = name
        self.width = kwargs.get("width", 60)
        self.height = kwargs.get("height", 40)
        self.threshold = kwargs.get("threshold", 1e-5)


    def plot(self, vector_field):
        # Create the meshgrid
        X, Y = np.meshgrid(np.linspace(-0.85, 0.85, num=self.width),
                            np.linspace(-0.65, 0.65, num=self.height))
        U = np.empty(X.shape)
        V = np.empty(X.shape)
        M = np.empty(X.shape)

        # Iterate through the meshgrid to calculate the Vector Field
        for i in range(len(X)):
            for j in range(len(X[i])):
                x = X[i][j]
                y = Y[i][j]
                vec = vector_field.compute(data.Pose2D(x,y))
                u, v = vec[0], vec[1]
                norm = np.linalg.norm((u,v))
                if norm > self.threshold:
                    M[i][j] = norm
                    U[i][j], V[i][j] = math.versor((u,v))

        # Set the quiver vectors
        plt.quiver(X, Y, U, V, M, units='dots', pivot='mid', scale=0.1)

        # Draw field
        plt.plot(GAME_FIELD[0], GAME_FIELD[1], c='k')

        # Set title
        plt.title(f'Vector Field Plot: {self.name}')

        # Set x, y boundary limits
        plt.xlim(-0.9, 0.9)
        plt.ylim(-0.7, 0.7)

        # Show plot with gird
        plt.grid()
        plt.show()
