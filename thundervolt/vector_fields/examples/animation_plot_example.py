import base_example

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

from thundervolt.core import math, data
from thundervolt.vector_fields import fields

field_data = data.FieldData()
repell_field = fields.RadialField(
    target = (-0.3,0),
    max_radius = 1.0,
    decay_radius = 0.3,
    repelling = True,
)
attract_field = fields.RadialField(
    target = (0.3,0),
    max_radius = 3.0,
    decay_radius = 0.3,
    repelling = False,
)

my_field = fields.VectorField()
my_field.add(repell_field)
my_field.add(attract_field)


X, Y = np.meshgrid(np.linspace(-0.75, 0.75, num=40),
                    np.linspace(-0.65, 0.65, num=30))
U = np.empty(X.shape)
V = np.empty(X.shape)
M = np.empty(X.shape)

fig, ax = plt.subplots(1,1)
Q = ax.quiver(X, Y, U, V, M, units='dots', pivot='mid', scale=0.1)

ax.set_xlim(-0.8, 0.8)
ax.set_ylim(-0.7, 0.7)

def update_quiver(num, Q, X, Y, M):
    """updates the horizontal and vertical vector components by a
    fixed increment on each frame
    """

    y = ((num % 50) - 25) / 40

    repell_field.target = (-0.3, y)
    attract_field.target = (0.3, -y)

    # Iterate through the meshgrid to calculate the Vector Field
    for i in range(len(X)):
        for j in range(len(X[i])):
            x = X[i][j]
            y = Y[i][j]
            vec = my_field.compute((x,y))
            u, v = vec[0], vec[1]
            norm = np.linalg.norm((u,v))
            if norm > 1e-5:
                M[i][j] = norm
                U[i][j], V[i][j] = math.versor((u,v))

    Q.set_UVC(U,V,M)

    return Q,

anim = animation.FuncAnimation(fig, update_quiver, fargs=(Q, X, Y, M),
                               interval=0, blit=True)
fig.tight_layout()

plt.show()
