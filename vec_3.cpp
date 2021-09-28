#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include"/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Sparse"
#include "/usr/include/eigen3/Eigen/Core"
#include "/usr/include/eigen3/Eigen/LU"
using namespace std;
using namespace Eigen;

int main(){
    Eigen::Matrix<double, 3, 3> A;
    A <<  0, 3, -3,
         -3, 0,  0,
          3, 0,  0;
    Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> eigensolver(A);
    Eigen::Matrix<double, 3, 3> P;
    P = eigensolver.eigenvectors().real();
    cout << P << endl;
    cout << eigensolver.eigenvalues() << endl;
    cout << eigensolver.eigenvectors() << endl;

    Eigen
}
