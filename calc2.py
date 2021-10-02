import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def func_lorenz(var, t, p, r, b):
    dxdt = -p*var[0] +p*var[1]
    dydt = -var[0]*var[2] +r*var[0] -var[1]
    dzdt = var[0]*var[1] -b*var[2]

    return [dxdt, dydt, dzdt]


def plot3d(t_list, var_list):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.set_xlabel("$x$")  
    ax.set_ylabel("$y$")  
    ax.set_zlabel("$z$")  
    ax.plot(var_list[:, 0], var_list[:, 1], var_list[:, 2])

    plt.show()


if (__name__ == '__main__'):
    t_list = np.linspace(0.0, 100.0, 10000)
    p = 10
    r = 28
    b = 8/3
    var_init = [0.1, 0.1, 0.1]
    var_list = odeint(func_lorenz, var_init, t_list, args=(p, r, b))
    print(var_list)

    plot3d(t_list, var_list)