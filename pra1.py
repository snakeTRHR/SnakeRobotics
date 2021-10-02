import numpy as np

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
    var_init = np.hstack((c_0, t_0, n_0, b_0));
    print(var_init[8])