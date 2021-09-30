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

double r = 1.0;
double h = 0.5;

double curvature(double _s){
    return r / (r * r + h * h);
    //return std::abs(sin(_s * 0.5)) / std::pow(cos(_s) *cos(_s) + 1, 3.0/2.0);
    //return 5.0 / 4.0;
}
double torsion(double _s){
    return h / (r * r + h * h);
    //return 1.0 / 4.0;
    //return 0.0;
    //return 0;
}
Eigen::Matrix<double, 3, 1> Func_1(double _s, Eigen::Matrix<double, 3, 1> input_1){
    return (curvature(_s) * input_1);
}
Eigen::Matrix<double, 3, 1> Func_2(double _s, Eigen::Matrix<double, 3, 1> input_1, Eigen::Matrix<double, 3, 1> input_2){
    return ((-1 * curvature(_s) * input_1) + (torsion(_s) * input_2));
}
Eigen::Matrix<double, 3, 1> Func_3(double _s, Eigen::Matrix<double, 3, 1> input_1){
    return (-1 * torsion(_s) * input_1);
}
Eigen::Matrix<double, 3, 1> Func_4(double _s, Eigen::Matrix<double, 3, 1> input_1){
    return (input_1);
}

int main(){
    //(e_r, e_p, e_s)
    Eigen::Matrix<double, 3, 1> C;
    Eigen::Matrix<double, 3, 1> T;
    Eigen::Matrix<double, 3, 1> N;
    Eigen::Matrix<double, 3, 1> B;
    //set initial value
    C << 0, 0, 0;
    T << 1, 0, 0;
    N << 0, 1, 0;
    B << 0, 0, 1;

    int n = 1000;
    std::vector<double> val_x(n), val_y(n), val_z(n);

    //4次ルンゲクッタ
    double h = 0.05;
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

    double s = 0;

    std::vector<double> C_x, C_y, C_z;
    std::vector<double> T_x, T_y, T_z;

    for(int i = 0; i < n; ++i){
        K_a_1 = h * Func_1(s, N);
        K_a_2 = h * Func_2(s, T, B);
        K_a_3 = h * Func_3(s, N);
        K_a_4 = h * Func_4(s, T);

        K_b_1 = h * Func_1(s + h / 2, N + K_a_2 / 2);
        K_b_2 = h * Func_2(s + h / 2, T + K_a_1 / 2, B + K_a_3 / 2);
        K_b_3 = h * Func_3(s + h / 2, N + K_a_2 / 2);
        K_b_4 = h * Func_4(s + h / 2, T + K_a_1 / 2);


        K_c_1 = h * Func_1(s + h / 2, N + K_b_2 / 2);
        K_c_2 = h * Func_2(s + h / 2, T + K_b_1 / 2, B + K_b_3 / 2);
        K_c_3 = h * Func_3(s + h / 2, N + K_b_2 / 2);
        K_c_4 = h * Func_4(s + h / 2, T + K_b_1 / 2);

        K_d_1 = h * Func_1(s + h, N + K_c_2);
        K_d_2 = h * Func_2(s + h, T + K_c_1, B + K_c_3);
        K_d_3 = h * Func_3(s + h, N + K_c_2);
        K_d_4 = h * Func_4(s + h, T + K_c_1);

        s += h;

        T += (K_a_1 + 2 * K_b_1 + 2 * K_c_1 + K_d_1) / 6;
        N += (K_a_2 + 2 * K_b_2 + 2 * K_c_2 + K_d_2) / 6;
        B += (K_a_3 + 2 * K_b_3 + 2 * K_c_3 + K_d_3) / 6;
        //C += (K_a_4 + 2 * K_b_4 + 2 * K_c_4 + K_d_4) / 6;

        T_x.push_back(T(0, 0));
        T_y.push_back(T(1, 0));
        T_z.push_back(T(2, 0));
    }

    //数値積分を行う
    //シンプソンの積分公式を用いる(区間[a,b]をn分割)
    for(int i = 0; i < n; ++i){   
        std::cout << "i = " << i << std::endl; 
        Eigen::Matrix<double, 3, 1> value;
        double start = 0, finish = (i * h) - h;
        double separate_num = 100;
        double width = (finish - start) / separate_num;
        double x = 0, y = 0, z = 0;
        for(int k = 0; k < separate_num; ++k){
            x = T_x[i] * width;
            y = T_x[i] * width;
            z = T_x[i] * width;
        }
        C_x.push_back(x);
        C_x.push_back(y);
        C_x.push_back(z);
    }

    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("label", "parametric curve") );
    //plt::plot3(C_x, C_y, C_z, keywords);
    //plt::plot(C_x, C_z);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::set_zlabel("z label"); 
    //plt::legend();
    plt::show();
}
