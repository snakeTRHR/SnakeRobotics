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

double r_c = 1.0;
double h_c = 0.5;

double curvature(double _s){
    //return r_c / (r_c * r_c + h_c * h_c);
    //return std::abs(sin(_s * 0.5)) / std::pow(cos(_s) *cos(_s) + 1, 3.0/2.0);
    return 1.0 / 4.0;
    //return sin(_s * M_PI);
    //return 0;
}
double torsion(double _s){
    //return h_c / (r_c * r_c + h_c * h_c);
    return std::sqrt(3) / 4.0;
    //return sin(_s * M_PI + M_PI / 2);
    //return 0;
}
Eigen::Matrix<double, 3, 1> Func_c(double _s, Eigen::Matrix<double, 3, 1> _E_1){
    return (_E_1);
}
Eigen::Matrix<double, 3, 1> Func_1(double _s, Eigen::Matrix<double, 3, 1> _E_2){
    return (curvature(_s) * _E_2);
}
Eigen::Matrix<double, 3, 1> Func_2(double _s, Eigen::Matrix<double, 3, 1> _E_1, Eigen::Matrix<double, 3, 1> _E_3){
    return ((-1 * curvature(_s) * _E_1) + (torsion(_s) * _E_3));
}
Eigen::Matrix<double, 3, 1> Func_3(double _s, Eigen::Matrix<double, 3, 1> _E_2){
    return (-1 * torsion(_s) * _E_2);
}

int main(){
    Eigen::Matrix<double, 3, 1> C;
    Eigen::Matrix<double, 3, 1> E_1;
    Eigen::Matrix<double, 3, 1> E_2;
    Eigen::Matrix<double, 3, 1> E_3;
    //set initial value
    C << 0, 0, 0;
    E_1 << 1, 0, 0;
    E_2 << 0, 1, 0;
    E_3 << 0, 0, 1;

    Eigen::Matrix<double, 3, 1> E_1_SUM;
    Eigen::Matrix<double, 3, 1> E_2_SUM;
    Eigen::Matrix<double, 3, 1> E_3_SUM;
    E_1_SUM << 1, 0, 0;
    E_2_SUM << 0, 1, 0;
    E_3_SUM << 0, 0, 1;
    
    
    //4次ルンゲクッタ
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

    double s_long = 70;
    double s = 0;
    double h = 0.05;
    double n = s_long / h;
    std::vector<double> C_x, C_y, C_z;
    std::vector<double> E_1_x, E_1_y, E_1_z;
    std::vector<double> E_2_x, E_2_y, E_2_z;
    std::vector<double> E_3_x, E_3_y, E_3_z;

    for(int i = 0; i < n; ++i){
        K_a_c = h * Func_c(s, E_1);
        K_a_1 = h * Func_1(s, E_2);
        K_a_2 = h * Func_2(s, E_1, E_3);
        K_a_3 = h * Func_3(s, E_2);

        K_b_c = h * Func_c(s + h / 2, E_1 + K_a_1 / 2);
        K_b_1 = h * Func_1(s + h / 2, E_2 + K_a_2 / 2);
        K_b_2 = h * Func_2(s + h / 2, E_1 + K_a_1 / 2, E_3 + K_a_3 / 2);
        K_b_3 = h * Func_3(s + h / 2, E_2 + K_a_2 / 2);


        K_c_c = h * Func_c(s + h / 2, E_1 + K_b_1 / 2);
        K_c_1 = h * Func_1(s + h / 2, E_2 + K_b_2 / 2);
        K_c_2 = h * Func_2(s + h / 2, E_1 + K_b_1 / 2, E_3 + K_b_3 / 2);
        K_c_3 = h * Func_3(s + h / 2, E_2 + K_b_2 / 2);

        K_d_c = h * Func_c(s + h, E_1 + K_c_1);
        K_d_1 = h * Func_1(s + h, E_2 + K_c_2);
        K_d_2 = h * Func_2(s + h, E_1 + K_c_1, E_3 + K_c_3);
        K_d_3 = h * Func_3(s + h, E_2 + K_c_2);

        s += h;

        //C   += (K_a_c + 2 * K_b_c + 2 * K_c_c + K_d_c) / 6;
        E_1 += (K_a_1 + 2 * K_b_1 + 2 * K_c_1 + K_d_1) / 6;
        E_2 += (K_a_2 + 2 * K_b_2 + 2 * K_c_2 + K_d_2) / 6;
        E_3 += (K_a_3 + 2 * K_b_3 + 2 * K_c_3 + K_d_3) / 6;
        
        std::cout << "E_1" << std::endl << E_1 << std::endl;
        std::cout << "E_2" << std::endl << E_2 << std::endl;
        std::cout << "E_3" << std::endl << E_3 << std::endl;
        
        E_1_SUM += E_1 * h;
        E_2_SUM += E_2 * h;
        E_3_SUM += E_3 * h;

        //C_x.push_back(C(0, 0));
        //C_y.push_back(C(1, 0));
        //C_z.push_back(C(2, 0));

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
    
    std::cout << "E_1_SUM" << std::endl << E_1_SUM << std::endl;
    std::cout << "E_2_SUM" << std::endl << E_2_SUM << std::endl;
    std::cout << "E_3_SUM" << std::endl << E_3_SUM << std::endl;

    //数値積分を行う
    double x = 0, y = 0, z = 0;
    for(int k = 0; k < n; ++k){
        x += E_1_x[k] * h;
        y += E_1_y[k] * h;
        z += E_1_z[k] * h;
        C_x.push_back(x);
        C_y.push_back(y);
        C_z.push_back(z);
    }
    
    //matplotlibで表示
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("label", "parametric curve") );
    plt::plot3(E_1_x, E_1_y, E_1_z, keywords);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::set_zlabel("z label"); 
    plt::legend();
    plt::show();
}
