import numpy as np
import matplotlib.pyplot as plt

from thundervolt.core import math
from thundervolt.vector_fields import fields

class FieldPlotter():
    def __init__(self, name, **kwargs):
        self.name = name
        self.width = kwargs.get("width", 60)
        self.height = kwargs.get("height", 40)
        self.threshold = kwargs.get("threshold", 1e-5)


    def plot(self, vector_field):
        # Create the meshgrid
        X, Y = np.meshgrid(np.linspace(-0.75, 0.75, num=self.width),
                            np.linspace(-0.65, 0.65, num=self.height))
        U = np.empty(X.shape)
        V = np.empty(X.shape)
        M = np.empty(X.shape)

        # Iterate through the meshgrid to calculate the Vector Field
        for i in range(len(X)):
            for j in range(len(X[i])):
                x = X[i][j]
                y = Y[i][j]
                u, v = vector_field.compute((x,y))
                norm = np.linalg.norm((u,v))
                if norm > self.threshold:
                    M[i][j] = norm
                    U[i][j], V[i][j] = math.versor((u,v))

        # Set the quiver vectors
        plt.quiver(X, Y, U, V, M, units='dots', pivot='tip', scale=0.1)

        # Set title
        plt.title(f'Vector Field Plot: {self.name}')

        # Set x, y boundary limits
        plt.xlim(-0.8, 0.8)
        plt.ylim(-0.7, 0.7)

        # Show plot with gird
        plt.grid()
        plt.show()
