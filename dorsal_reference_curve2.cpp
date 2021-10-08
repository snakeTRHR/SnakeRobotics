#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Sparse"
#include "/usr/include/eigen3/Eigen/Core"
#include "/usr/include/eigen3/Eigen/LU"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

double alpha_yaw = M_PI / 4;
double alpha_pitch = 0;
double bias_yaw = 0;
double bias_pitch = 0;
double l = 5;

double curvature_yaw(double _s){
    return alpha_yaw * M_PI * sin(_s * M_PI/ (2 * l))/ (2 * l) + bias_yaw;
}
double curvature_pitch(double _s){
    return 0;
    //return alpha_pitch * M_PI * sin(4* M_PI * _s * M_PI / (2 * l))/ (2 * l) + bias_pitch;
}
double torsion(double _s){
    return 0;
}
Eigen::Matrix<double, 3, 1> Func_C(double _s, Eigen::Matrix<double, 3, 1> input_1){
    return (input_1);
}
Eigen::Matrix<double, 3, 1> Func_E_r(double _s, Eigen::Matrix<double, 3, 1> input_1, Eigen::Matrix<double, 3, 1> input_2){
    return (curvature_yaw(_s) * input_1 - curvature_pitch(_s) * input_2);
}
Eigen::Matrix<double, 3, 1> Func_E_p(double _s, Eigen::Matrix<double, 3, 1> input_1, Eigen::Matrix<double, 3, 1> input_2){
    return (-1 * curvature_yaw(_s) * input_1 + torsion(_s) * input_2);
}
Eigen::Matrix<double, 3, 1> Func_E_y(double _s, Eigen::Matrix<double, 3, 1> input_1, Eigen::Matrix<double, 3, 1> input_2){
    return (curvature_pitch(_s) * input_1 - torsion(_s) * input_2);
}

int main(){
    //(e_r, e_p, e_s)
    Eigen::Matrix<double, 3, 1> C;
    Eigen::Matrix<double, 3, 1> E_r;
    Eigen::Matrix<double, 3, 1> E_p;
    Eigen::Matrix<double, 3, 1> E_y;
    //set initial value
    C   << 0, 0, 0;
    //E_r << 1, 0, 0;
    //E_p << 0, 1, 0;
    //E_y << 0, 0, 1;
    Eigen::Matrix<double, 3, 3> Identity_Matrix;
    Identity_Matrix << 1,  0,  0,
                       0, -1,  0,
                       0,  0, -1;

    double theta_roll = 0;
    double theta_pitch = alpha_pitch;
    double theta_yaw = alpha_yaw;

    Eigen::Matrix<double, 3, 3> Rotation_Matrix;
    //Rotation_Matrix <<  cos(theta_pitch) * cos(theta_roll), sin(theta_yaw) * sin(theta_pitch) * cos(theta_roll),  sin(theta_yaw) * sin(theta_roll) + cos(theta_yaw) * sin(theta_pitch) * cos(theta_roll),
    //                    cos(theta_pitch) * sin(theta_roll), sin(theta_yaw) * sin(theta_pitch) * sin(theta_roll), -sin(theta_yaw) * cos(theta_roll) + cos(theta_yaw) * sin(theta_pitch) * sin(theta_roll),
    //                   -sin(theta_pitch)                  , sin(theta_yaw) * cos(theta_pitch)                  ,  cos(theta_yaw) * cos(theta_pitch);
    Rotation_Matrix <<  cos(theta_pitch) * cos(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * sin(theta_yaw), -cos(theta_pitch) * sin(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  sin(theta_pitch) * cos(theta_roll),
                        cos(theta_roll)  * sin(theta_yaw)                                                      ,  cos(theta_roll)  * cos(theta_yaw)                                                      , -sin(theta_roll),
                       -sin(theta_pitch) * cos(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * sin(theta_yaw),  sin(theta_pitch) * sin(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  cos(theta_pitch) * cos(theta_roll);

    Eigen::Matrix<double, 3, 3> Initial_Matrix;
    Initial_Matrix = Rotation_Matrix * Identity_Matrix;
    Initial_Matrix.transposeInPlace();

    E_r = Initial_Matrix.row(0);
    E_p = Initial_Matrix.row(1);
    E_y = Initial_Matrix.row(2);

    std::cout << "E_r" << std::endl << E_r << std::endl; 
    std::cout << "E_p" << std::endl << E_p << std::endl; 
    std::cout << "E_y" << std::endl << E_y << std::endl; 
    //RungeKutta
    Eigen::Matrix<double, 3, 1> K_a_1;
    Eigen::Matrix<double, 3, 1> K_a_2;
    Eigen::Matrix<double, 3, 1> K_a_3;
    Eigen::Matrix<double, 3, 1> K_a_4;

    Eigen::Matrix<double, 3, 1> K_b_1;
    Eigen::Matrix<double, 3, 1> K_b_2;
    Eigen::Matrix<double, 3, 1> K_b_3;
    Eigen::Matrix<double, 3, 1> K_b_4;

    Eigen::Matrix<double, 3, 1> K_c_1;
    Eigen::Matrix<double, 3, 1> K_c_2;
    Eigen::Matrix<double, 3, 1> K_c_3;
    Eigen::Matrix<double, 3, 1> K_c_4;

    Eigen::Matrix<double, 3, 1> K_d_1;
    Eigen::Matrix<double, 3, 1> K_d_2;
    Eigen::Matrix<double, 3, 1> K_d_3;
    Eigen::Matrix<double, 3, 1> K_d_4;

    double s_long = 4 * l;
    double s = 0;
    double h = 0.05;
    double n = s_long / h;
    std::vector<double> C_x, C_y, C_z;

    for(int i = 0; i < n; ++i){
        K_a_1 = h * Func_C(s, E_r);
        K_a_2 = h * Func_E_r(s, E_p, E_y);
        K_a_3 = h * Func_E_p(s, E_r, E_y);
        K_a_4 = h * Func_E_y(s, E_r, E_p);

        K_b_1 = h *   Func_C(s + h / 2, E_r + K_a_2 / 2);
        K_b_2 = h * Func_E_r(s + h / 2, E_p + K_a_3 / 2, E_y + K_a_4 / 2);
        K_b_3 = h * Func_E_p(s + h / 2, E_r + K_a_2 / 2, E_y + K_a_4 / 2);
        K_b_4 = h * Func_E_y(s + h / 2, E_r + K_a_2 / 2, E_p + K_a_3 / 2);

        K_c_1 = h *   Func_C(s + h / 2, E_p + K_b_2 / 2);
        K_c_2 = h * Func_E_r(s + h / 2, E_p + K_b_3 / 2, E_y + K_b_4 / 2);
        K_c_3 = h * Func_E_p(s + h / 2, E_r + K_b_2 / 2, E_y + K_b_4 / 2);
        K_c_4 = h * Func_E_y(s + h / 2, E_r + K_b_2 / 2, E_p + K_b_3 / 2);

        K_d_1 = h *   Func_C(s + h, E_r + K_c_2);
        K_d_2 = h * Func_E_r(s + h, E_p + K_c_3, E_y + K_c_4);
        K_d_3 = h * Func_E_p(s + h, E_r + K_c_2, E_y + K_c_4);
        K_d_4 = h * Func_E_y(s + h, E_r + K_c_2, E_p + K_c_3);

        s += h;

        C   += (K_a_1 + 2 * K_b_1 + 2 * K_c_1 + K_d_1) / 6;
        E_r += (K_a_2 + 2 * K_b_2 + 2 * K_c_2 + K_d_2) / 6;
        E_p += (K_a_3 + 2 * K_b_3 + 2 * K_c_3 + K_d_3) / 6;
        E_y += (K_a_4 + 2 * K_b_4 + 2 * K_c_4 + K_d_4) / 6;

        C_x.push_back(C(0, 0));
        C_y.push_back(C(1, 0));
        C_z.push_back(C(2, 0));
    }
    std::cout << "s = " << s << std::endl;
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("label", "parametric curve") );

    plt::plot3(C_x, C_y, C_z, keywords);
    //plt::plot(C_x, C_y);
    plt::xlabel("x label");
    plt::ylabel("y label");
    //plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();
    plt::show();
}
