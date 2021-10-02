import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

r_c = 1.0
h_c = 0.5
def func_frenet_serret(var , s, curvature, torsion):
    T = [var[3],  var[4],  var[5]]
    N = [var[6],  var[7],  var[8]]
    B = [var[9], var[10], var[11]]
    dCds = T
    dTds = np.dot(curvature,N)
    dNds = np.dot(-1 * curvature, T) + np.dot(torsion, B)
    dBds = np.dot(-1 * torsion, N)
    print(dNds)
    ans = np.hstack((dCds, dTds, dNds, dBds))
    print(ans)
    return ans

def plot3d(t_list, var_list):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.set_xlabel("$x$")  
    ax.set_ylabel("$y$")  
    ax.set_zlabel("$z$")  
    ax.plot(var_list[:, 0], var_list[:, 1], var_list[:, 2])

    plt.show()


if (__name__ == '__main__'):
    s_list = np.linspace(0, 30.0, 10000)
    r = 1.0;
    h = 0.5;
    curvature = r / (r * r + h * h);
    torsion = h / (r * r + h * h);
    c_0 = [0, 0, 0]
    t_0 = [1, 0, 0]
    n_0 = [0, 1, 0]
    b_0 = [0, 0, 1]
    var_init = np.hstack((c_0, t_0, n_0, b_0))
    var_list = odeint(func_frenet_serret, var_init, s_list, args=(curvature, torsion))
    print(var_list)

    plot3d(s_list, var_list)