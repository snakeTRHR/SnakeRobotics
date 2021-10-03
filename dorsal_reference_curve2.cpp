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

double alpha_yaw = M_PI / 6;
double alpha_pitch = M_PI / 9;
double bias_yaw = 0;
double bias_pitch = 0;
double l = 2;

double curvature_yaw(double _s){
    return -2 * 5 * alpha_yaw * M_PI * sin(2 * 5 * M_PI * _s / l)/ l + bias_yaw;
    //return -1 * alpha_yaw * M_PI * sin(M_PI * _s / (2 * l))/ (2 * l) + bias_yaw;
}
double curvature_pitch(double _s){
    //return alpha_pitch * M_PI * sin(_s * M_PI / (2 * l))/ (2 * l) + bias_pitch;
    return 0;
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
    E_r << 1, 0, 0;
    E_p << 0, 1, 0;
    E_y << 0, 0, 1;

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

    double s_long = 6;
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
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("label", "parametric curve") );

    //plt::plot3(C_x, C_y, C_z, keywords);
    plt::plot(C_x, C_y);
    plt::xlabel("x label");
    plt::ylabel("y label");
    //plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();
    plt::show();
}
