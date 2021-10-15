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
    //横うねり推進(岡山大学論文参照)
    return alpha_yaw * M_PI * sin(_s * M_PI/ (2 * l))/ (2 * l) + bias_yaw;
}
double curvature_pitch(double _s){
    return 0;
}
double torsion(double _s){
    return 0;
}
Eigen::Matrix<double, 3, 1> Func_c(double _s, Eigen::Matrix<double, 3, 1> _E_1){
    return (_E_1);
}
Eigen::Matrix<double, 3, 1> Func_1(double _s, Eigen::Matrix<double, 3, 1> _E_2, Eigen::Matrix<double, 3, 1> _E_3){
    return (curvature_yaw(_s) * _E_2 - curvature_pitch(_s) * _E_3);
}
Eigen::Matrix<double, 3, 1> Func_2(double _s, Eigen::Matrix<double, 3, 1> _E_1, Eigen::Matrix<double, 3, 1> _E_3){
    return ((-1 * curvature_yaw(_s) * _E_1) + (torsion(_s) * _E_3));
}
Eigen::Matrix<double, 3, 1> Func_3(double _s, Eigen::Matrix<double, 3, 1> _E_1, Eigen::Matrix<double, 3, 1> _E_2){
    return (curvature_pitch(_s) * _E_1 -1 * torsion(_s) * _E_2);
}

int main(){
    Eigen::Matrix<double, 3, 1> C;
    Eigen::Matrix<double, 3, 1> E_1;
    Eigen::Matrix<double, 3, 1> E_2;
    Eigen::Matrix<double, 3, 1> E_3;
    //初期値の設定
    C << 0, 0, 0;

    Eigen::Matrix<double, 3, 3> Identity_Matrix;
    Identity_Matrix << 1,  0,  0,
                       0, -1,  0,
                       0,  0, -1;

    double theta_roll = 0;
    double theta_pitch = -alpha_pitch;
    double theta_yaw = alpha_yaw;

    Eigen::Matrix<double, 3, 3> Rotation_Matrix;
    Rotation_Matrix <<  cos(theta_pitch) * cos(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * sin(theta_yaw), -cos(theta_pitch) * sin(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  sin(theta_pitch) * cos(theta_roll),
                        cos(theta_roll)  * sin(theta_yaw)                                                      ,  cos(theta_roll)  * cos(theta_yaw)                                                      , -sin(theta_roll),
                       -sin(theta_pitch) * cos(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * sin(theta_yaw),  sin(theta_pitch) * sin(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  cos(theta_pitch) * cos(theta_roll);

    Eigen::Matrix<double, 3, 3> Initial_Matrix;
    Initial_Matrix = Rotation_Matrix * Identity_Matrix;
    Initial_Matrix.transposeInPlace();

    E_1 = Initial_Matrix.row(0);
    E_2 = Initial_Matrix.row(1);
    E_3 = Initial_Matrix.row(2);
    
    std::cout << "E_1" << std::endl << E_1 << std::endl; 
    std::cout << "E_2" << std::endl << E_2 << std::endl; 
    std::cout << "E_3" << std::endl << E_3 << std::endl; 
    
    //４次ルンゲクッタで連立微分方程式を解く
    //http://skomo.o.oo7.jp/f20/hp20_4-3.htm
    Eigen::Matrix<double, 3, 1> K_a_c;
    Eigen::Matrix<double, 3, 1> K_a_1;
    Eigen::Matrix<double, 3, 1> K_a_2;
    Eigen::Matrix<double, 3, 1> K_a_3;

    Eigen::Matrix<double, 3, 1> K_b_c;
    Eigen::Matrix<double, 3, 1> K_b_1;
    Eigen::Matrix<double, 3, 1> K_b_2;
    Eigen::Matrix<double, 3, 1> K_b_3;

    Eigen::Matrix<double, 3, 1> K_c_c;
    Eigen::Matrix<double, 3, 1> K_c_1;
    Eigen::Matrix<double, 3, 1> K_c_2;
    Eigen::Matrix<double, 3, 1> K_c_3;

    Eigen::Matrix<double, 3, 1> K_d_c;
    Eigen::Matrix<double, 3, 1> K_d_1;
    Eigen::Matrix<double, 3, 1> K_d_2;
    Eigen::Matrix<double, 3, 1> K_d_3;

    double s_long = 30;
    double s = 0;
    double h = 0.05;
    double n = s_long / h;
    std::vector<double> C_x, C_y, C_z;
    std::vector<double> E_1_x, E_1_y, E_1_z;
    std::vector<double> E_2_x, E_2_y, E_2_z;
    std::vector<double> E_3_x, E_3_y, E_3_z;

    for(int i = 0; i < n; ++i){
        K_a_c = h * Func_c(s, E_1);
        K_a_1 = h * Func_1(s, E_2, E_3);
        K_a_2 = h * Func_2(s, E_1, E_3);
        K_a_3 = h * Func_3(s, E_1, E_2);

        K_b_c = h * Func_c(s + h / 2, E_1 + K_a_1 / 2);
        K_b_1 = h * Func_1(s + h / 2, E_2 + K_a_2 / 2, E_3 + K_a_3 / 2);
        K_b_2 = h * Func_2(s + h / 2, E_1 + K_a_1 / 2, E_3 + K_a_3 / 2);
        K_b_3 = h * Func_3(s + h / 2, E_1 + K_a_1 / 2, E_2 + K_a_2 / 2);


        K_c_c = h * Func_c(s + h / 2, E_1 + K_b_1 / 2);
        K_c_1 = h * Func_1(s + h / 2, E_2 + K_b_2 / 2, E_3 + K_b_3 / 2);
        K_c_2 = h * Func_2(s + h / 2, E_1 + K_b_1 / 2, E_3 + K_b_3 / 2);
        K_c_3 = h * Func_3(s + h / 2, E_1 + K_a_1 / 2, E_2 + K_b_2 / 2);

        K_d_c = h * Func_c(s + h, E_1 + K_c_1);
        K_d_1 = h * Func_1(s + h, E_2 + K_c_2, E_3 + K_c_3);
        K_d_2 = h * Func_2(s + h, E_1 + K_c_1, E_3 + K_c_3);
        K_d_3 = h * Func_3(s + h, E_1 + K_c_1, E_2 + K_c_2);

        s += h;

        C   += (K_a_c + 2 * K_b_c + 2 * K_c_c + K_d_c) / 6;
        E_1 += (K_a_1 + 2 * K_b_1 + 2 * K_c_1 + K_d_1) / 6;
        E_2 += (K_a_2 + 2 * K_b_2 + 2 * K_c_2 + K_d_2) / 6;
        E_3 += (K_a_3 + 2 * K_b_3 + 2 * K_c_3 + K_d_3) / 6;
        
        C_x.push_back(C(0, 0));
        C_y.push_back(C(1, 0));
        C_z.push_back(C(2, 0));

        E_1_x.push_back(E_1(0, 0));
        E_1_y.push_back(E_1(1, 0));
        E_1_z.push_back(E_1(2, 0));
        
        E_2_x.push_back(E_2(0, 0));
        E_2_y.push_back(E_2(1, 0));
        E_2_z.push_back(E_2(2, 0));
        
        E_3_x.push_back(E_3(0, 0));
        E_3_y.push_back(E_3(1, 0));
        E_3_z.push_back(E_3(2, 0));
    }
    
    //matplotlibで表示
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("label", "parametric curve") );
    plt::plot(C_x, C_y, keywords);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::legend();
    plt::show();
}
