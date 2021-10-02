import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

r_c = 1.0
h_c = 0.5

def curvature(_s):
    return r_c / (r_c * r_c + h_c * h_c)

def torsion(_s):
    return h_c / (r_c * r_c + h_c * h_c)

def func_frenet_serret(var , s):
    A = np.array([[                1,                  0,          0]       
                  [                0,       curvature(s),          0],
                  [-1 * curvature(s),                  0, torsion(s)],
                  [                 0, -1 * curvature(s),          0]])
    F = np.array([[var[0],  var[1],  var[2]],
                  [var[3],  var[4],  var[5]],
                  [var[6],  var[7],  var[8]],
                  [var[9], var[10], var[11]]])
    dFds = np.dot(A, F)
    return [dFds]


def plot3d(t_list, var_list):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.set_xlabel("$x$")  
    ax.set_ylabel("$y$")  
    ax.set_zlabel("$z$")  
    ax.plot(var_list[:, 0], var_list[:, 1], var_list[:, 2])

    plt.show()


if (__name__ == '__main__'):
    s_list = np.linspace(0.0, 100.0, 10000)
    c_0 = [0, 0, 0]
    t_0 = [1, 0, 0]
    n_0 = [0, 1, 0]
    b_0 = [0, 0, 1]
    var_init = np.hstack((c_0, t_0, n_0, b_0));
    print(var_init)
    var_list = odeint(func_frenet_serret, var_init, s_list)
    print(var_list)

    plot3d(s_list, var_list)