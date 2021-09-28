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

double curvature_y(int input){
    //return sin(input * 0.01);
    return 3.0;
}
double curvature_p(int input){
    //return sin(input * 0.01);
    return 3.0;
}
double torsion(){
}

int main(){
    //(e_r, e_p, e_s)
    Eigen::Matrix<double, 3, 1> c_s;
    Eigen::Matrix<double, 3, 1> e_r;
    Eigen::Matrix<double, 3, 1> e_p;
    Eigen::Matrix<double, 3, 1> e_s;
    //set initial value
    c_s << 0,
           0,
           0;
    e_r << 1,
           0,
           0;
    e_p << 0,
           1,
           0;
    e_s << 0,
           0,
           1;

    int n = 10;
    std::vector<double> val_x(n), val_y(n), val_z(n);

    for(int s = 0; s < n; ++s){
        //solve differential equation(using diagonalization)
        //https://www.momoyama-usagi.com/entry/math-ode11
        //d/dx = Ax
        //diagonalization
        Eigen::Matrix<double, 3, 3> A;
        A <<             0, curvature_y(s), -1 * curvature_p(s),
             -curvature_y(s),            0,                 0,
              curvature_p(s),            0,                 0;
        std::cout << "A" << std::endl;
        std::cout << A << std::endl;
        Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> eigensolver(A);
        Eigen::Matrix<double, 3, 3> P;
        P = eigensolver.eigenvectors().real();
        std::cout << "P" << std::endl;
        std::cout << P << std::endl;
        Eigen::Matrix<double, 3, 3> D;
        Eigen::Matrix<double, 3, 3> P_INV;
        P_INV = P.inverse();
        std::cout << "P_INV" << std::endl;
        std::cout << P_INV << std::endl;
        D = P_INV * A * P;
        std::cout << "D" << std::endl;
        std::cout << D << std::endl;
        //P^-1x = y
        //d/dty = Dy
        Eigen::Matrix<double, 3, 3> Y;
        Y << exp(D(0, 0)), exp(D(0, 1)), exp(D(0, 2)),
             exp(D(1, 0)), exp(D(1, 1)), exp(D(1, 2)),
             exp(D(2, 0)), exp(D(2, 1)), exp(D(2, 2));
        //x = Py
        Eigen::Matrix<double, 3, 3> X;
        X = P * Y;

        Eigen::Matrix<double, 3, 1> C;
        //FIX
        C += X.row(0);
        //display graph
        //set display value
        val_x[s] = C(0, 0);
        val_y[s] = C(1, 0);
        val_z[s] = C(2, 0);

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
