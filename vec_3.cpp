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
  Eigen::Matrix<double, 3, 1> C;
    Eigen::Matrix<double, 3, 1> E_1;
    Eigen::Matrix<double, 3, 1> E_2;
    Eigen::Matrix<double, 3, 1> E_3;
  
    Eigen::Matrix<double, 3, 3> Identity_Matrix;
    Identity_Matrix << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;

    double theta_roll = 0;
    double theta_pitch = M_PI / 4;
    double theta_yaw = 0;

    //Eigen::Matrix<double, 3, 3> Rotation_Matrix;
    //Rotation_Matrix <<  cos(theta_pitch) * cos(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * sin(theta_yaw), -cos(theta_pitch) * sin(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  sin(theta_pitch) * cos(theta_roll),
    //                    cos(theta_roll)  * sin(theta_yaw)                                                      ,  cos(theta_roll)  * cos(theta_yaw)                                                      , -sin(theta_roll),
    //                   -sin(theta_pitch) * cos(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * sin(theta_yaw),  sin(theta_pitch) * sin(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  cos(theta_pitch) * cos(theta_roll);
    Eigen::Matrix<double, 3, 3> Rotation_Matrix_X;
    Eigen::Matrix<double, 3, 3> Rotation_Matrix_Y;
    Eigen::Matrix<double, 3, 3> Rotation_Matrix_Z;

    Rotation_Matrix_X << 1,               0,                0,
                         0, cos(theta_roll), -sin(theta_roll),
                         0, sin(theta_roll),  cos(theta_roll);
    Rotation_Matrix_Y << cos(theta_pitch), 0,  sin(theta_pitch),
                                        0, 1,                 0,
                        -sin(theta_pitch), 0,  cos(theta_pitch); 
    Rotation_Matrix_Z << cos(theta_yaw), -sin(theta_yaw), 0,
                         sin(theta_yaw),  cos(theta_yaw), 0,
                                      0,               0, 1;
    Eigen::Matrix<double, 3, 3> Initial_Matrix;
    //Initial_Matrix = Rotation_Matrix * Identity_Matrix;
    Initial_Matrix = Rotation_Matrix_Y * Identity_Matrix;
    Initial_Matrix.transposeInPlace();

    E_1 = Initial_Matrix.row(0);
    E_2 = Initial_Matrix.row(1);
    E_3 = Initial_Matrix.row(2);

    cout << "E_1" << endl << E_1 << endl;
    cout << "E_2" << endl << E_2 << endl;
    cout << "E_3" << endl << E_3 << endl;
}
