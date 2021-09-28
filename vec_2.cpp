#include<iostream>
#include"/usr/include/eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;
int main(){
   Matrix<double, 3, 3> A;
   A << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;
   //double a = A(2, 3);
   //cout << a << endl;
    //MatrixXd A = MatrixXd::Identity(3,3);
    cout << "A(0,0,0) = " << A.col(0) << endl;
    Matrix<double, 3, 1> B;
    B = A.col(0);
    cout << B << endl;
}
