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
    cout << "A(0,0,0) = " << A(2,1) << endl;;
    cout << "A.coeff(1,2) = " << A.coeff(1,2) << endl;
    cout << "A.coeffRef(1,2) = " << A.coeff(1,2) << endl;
 
}
