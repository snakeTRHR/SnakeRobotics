import numpy as np


r_c = 1.0
h_c = 0.5
def curvature():
    return r_c / (r_c * r_c + h_c * h_c)

def torsion():
    return h_c / (r_c * r_c + h_c * h_c)

if (__name__ == '__main__'):
    A = np.array([[1, 2],
                  [3, 4]])
    x = np.array([[5],
                  [6]])
    ans = np.dot(A, x)
    print(ans)

    c_0 = [0, 0, 0]
    t_0 = [1, 0, 0]
    n_0 = [0, 1, 0]
    b_0 = [0, 0, 1]
    var = np.hstack((c_0, t_0, n_0, b_0));
    T = [[var[0], var[1], var[2]]]
    N = [[var[3], var[4], var[5]]]
    B = [[var[6], var[7], var[8]]]
    dCds = T
    dTds = np.dot(curvature(),N)
    dNds = np.dot(-1 * curvature(), T) + np.dot(torsion(), B)
    dBds = np.dot(-1 * torsion(), N)