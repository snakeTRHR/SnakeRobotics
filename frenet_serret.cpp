#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include "matplotlibcpp.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Sparse"
#include "/usr/include/eigen3/Eigen/Core"
#include "/usr/include/eigen3/Eigen/LU"

namespace plt = matplotlibcpp;

double a = 1;
double b = 0.1;

double curvature(){
    return a /(a * a + b * b);
}
double torsion(){
    return b / (a * a + b *b);
}
Eigen::Matrix<double, 3, 1> Func_1(Eigen::Matrix<double, 3, 1> input_1){
    return (curvature() * input_1);
}
Eigen::Matrix<double, 3, 1> Func_1(Eigen::Matrix<double, 3, 1> input_1, Eigen::Matrix<double, 3, 1> input_2){
    return (-1 * curvature() * input_1 + torsion * input_2);
}
Eigen::Matrix<double, 3, 1> Func_1(Eigen::Matrix<double, 3, 1> input_1){
    return (-1 * input_2);
}

int main(){
    //(e_r, e_p, e_s)
    Eigen::Matrix<double, 3, 1> c_s;
    Eigen::Matrix<double, 3, 1> T;
    Eigen::Matrix<double, 3, 1> N;
    Eigen::Matrix<double, 3, 1> B;
    //set initial value
    c_s << 0, 0, 0;
    T << 1, 0, 0;
    N << 0, 1, 0;
    B << 0, 0, 1;

    int n = 10;
    std::vector<double> val_x(n), val_y(n), val_z(n);

    //RungeKutta
    double h = 0.05;
    Eigen::Matrix<double, 3, 1> K_a_1;
    Eigen::Matrix<double, 3, 1> K_a_2;
    Eigen::Matrix<double, 3, 1> K_a_3;

    Eigen::Matrix<double, 3, 1> K_b_1;
    Eigen::Matrix<double, 3, 1> K_b_2;
    Eigen::Matrix<double, 3, 1> K_b_3;

    Eigen::Matrix<double, 3, 1> K_c_1;
    Eigen::Matrix<double, 3, 1> K_c_2;
    Eigen::Matrix<double, 3, 1> K_c_3;

    Eigen::Matrix<double, 3, 1> K_d_1;
    Eigen::Matrix<double, 3, 1> K_d_2;
    Eigen::Matrix<double, 3, 1> K_d_3;

    double t = 0;
    for(int s = 0; s < n; ++s){
        K_a_1 = h * Func_1(N);
        K_a_2 = h * Func_2(T, B);
        K_a_3 = h * Func_3(N);

        K_b_1 = h * Func_1(N + K_a_2 / 2);
        K_b_2 = h * Func_2(T + K_a_1 / 2, B + K_a_3 / 2);
        K_b_3 = h * Func_3(N + K_a_2 / 2);


        K_c_1 = h * Func_1(N + K_b_2 / 2);
        K_c_2 = h * Func_2(T + K_b_1 / 2, B + K_b_3 / 2);
        K_c_3 = h * Func_3(N + K_b_2 / 2);

        K_d_1 = h * Func_1(N + K_c_2);
        K_d_2 = h * Func_2(T + K_c_1, B + K_c_3);
        K_d_3 = h * Func_3(N + K_c_2);

        t = t + h;

        T += (K_a_1 + 2 * K_b_1 + 2 * K_c_1 + K_d_1) / 6;
        N += (K_a_2 + 2 * K_b_2 + 2 * K_c_2 + K_d_2) / 6;
        B += (K_a_3 + 2 * K_b_3 + 2 * K_c_3 + K_d_3) / 6;

        std::cout << T << std::endl;
    }

        std::cout << val_x[s] << " " << val_y[s] << " " << val_z[s] << std::endl;
        /*
 // Clear previous plot
		plt::clf();
		// Plot line from given x and y data. Color is selected automatically.
		plt::plot(val_x, val_y);
		// Plot a line whose name will show up as "log(x)" in the legend.
		plt::named_plot("serpenoid", val_x, val_z);
		// Set x-axis to interval [0,1000000]
		plt::xlim(-10, 10);
		// Add graph title
		plt::title("Sample figure");
		// Enable legend.
		plt::legend();
		// Display plot continuously
		plt::pause(0.01);*/
    }

}
