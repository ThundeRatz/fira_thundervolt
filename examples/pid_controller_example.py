import base_example
import matplotlib.pyplot as plt
import numpy as np
from thundervolt.core.pid_controller import pidController


if __name__ == "__main__":

    class FirstOrderSystem:
        def __init__(self, T, X0=0, dt=1):
            self.T = T
            self.X = X0
            self.dt = dt

        def step(self, u):
            X_dot = (u - self.X)/self.T
            self.X += X_dot*self.dt
            return self.X

    def run_fos_test(kp, ki, kd, tf=5, dt=0.01, T=1, X0=0):

        N = int(tf//dt)
        un_fos = FirstOrderSystem(T, X0, dt)
        con_fos = FirstOrderSystem(T, X0, dt)

        controller = pidController(
            kp, ki, kd, 1, 1/dt, integral_fade_rate=0.7)

        con_X = np.empty(N)
        un_X = np.empty(N)
        effort = np.empty(N)
        tspan = np.empty(N)

        con_X[0] = X0
        un_X[0] = X0
        tspan[0] = 0
        for i in range(1, N):
            tspan[i] = i*dt
            un_X[i] = un_fos.step(1)
            effort[i] = controller.update(con_X[i-1])
            con_X[i] = con_fos.step(effort[i])
        effort[0] = effort[1]

        plt.plot(tspan, un_X, 'b', label='Uncontrolled')
        plt.plot(tspan, con_X, 'r', label='Controlled')
        plt.plot(tspan, effort/10, 'orange', label='Effort')
        plt.legend()

    plt.subplot(221)
    run_fos_test(5, 0, 0)

    plt.subplot(222)
    run_fos_test(5, 0.8, 0)

    plt.subplot(223)
    run_fos_test(4, 0.8, 1e-1)

    plt.show()
