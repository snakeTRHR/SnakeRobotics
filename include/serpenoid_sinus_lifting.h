#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include<chrono>
#include<fstream>
#include<iomanip>
#include"/usr/include/eigen3/Eigen/Dense"
#include"/usr/include/eigen3/Eigen/Sparse"
#include"/usr/include/eigen3/Eigen/Core"
#include"/usr/include/eigen3/Eigen/LU"
#include"matplotlibcpp.h"

using namespace std::chrono;
namespace plt = matplotlibcpp;

class SnakeRobot{
    public:
        SnakeRobot(double _length, double _alpha_yaw, double _alpha_pitch):length(_length), alpha_yaw(_alpha_yaw), alpha_pitch(_alpha_pitch){
            //alpha_yawは初めの角度(普通はM_PI/4)(平面の場合はalpha_pitchは0)
            double theta_roll = 0;
            double theta_pitch = -alpha_pitch;
            double theta_yaw = alpha_yaw;
            C << 0, 0, 0;
            Identity_Matrix << 1,  0,  0,
                               0, -1,  0,
                               0,  0, -1;
            Eigen::Matrix<double, 3, 3> Rotation_Matrix;
            //回転行列(固定された軸回り(三次元運動するなら任意の軸回りに変える必要ありそう))
            Rotation_Matrix <<  cos(theta_pitch) * cos(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * sin(theta_yaw), -cos(theta_pitch) * sin(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  sin(theta_pitch) * cos(theta_roll),
                                cos(theta_roll)  * sin(theta_yaw)                                                      ,  cos(theta_roll)  * cos(theta_yaw)                                                      , -sin(theta_roll)                   ,
                               -sin(theta_pitch) * cos(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * sin(theta_yaw),  sin(theta_pitch) * sin(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  cos(theta_pitch) * cos(theta_roll);
            Initial_Matrix = Rotation_Matrix * Identity_Matrix;
            Initial_Matrix.transposeInPlace();
            E_1 = Initial_Matrix.row(0);
            E_2 = Initial_Matrix.row(1);
            E_3 = Initial_Matrix.row(2);
        }
        void Forward(){
            bias_yaw = 0;
            Update();
        }
        void Right(){
            bias_yaw = 0.025;
            Update();
        }
        void Left(){
            bias_yaw = -0.05;
            Update();
        }  
        void Update(){
            SolveDE();
            std::cout<<bias_yaw<<std::endl;
        }
        void changeBiasYaw(double _bias){
            bias_yaw = _bias;
        }
        void changeBiasPitch(double _bias){
            bias_pitch = _bias;
        }
        void changeVel(double _s_vel){
            s_vel = _s_vel;
        }
        void changeAlphaYaw(double _alpha_yaw){
            alpha_yaw = _alpha_yaw;
        }
        void changeAlphaPitch(double _alpha_pitch){
            alpha_pitch = _alpha_pitch;
        }
        void Animation(){
            //matplotlibで表示
            plt::clf();
            plt::plot(C_x, C_y);
            std::cout<<C_x.size()<<" "<<C_y.size()<<std::endl;
            std::cout<<C_x.back()<<" "<<C_y.back()<<std::endl;
            plt::named_plot("snake", C_x, C_y);
            //plt::xlim(0, 300);
            //plt::ylim(-10, 10);
            plt::legend();
            plt::pause(0.01);
        }
        void Clear(){
            s=0;
            C=Eigen::Vector3d::Zero();;
            C_x.clear();
            C_y.clear();
            C_z.clear();
        }
        double calL(){
            double L = 0;
            for(int i = 0; i < 4 * length; ++i){
                Update();
                Animation();
            }
            L=C_x.back();
            Clear();
            return L;
        }
        std::vector<double> C_x, C_y, C_z;
    private:
        Eigen::Matrix<double, 3, 1> C;
        Eigen::Matrix<double, 3, 1> E_1;
        Eigen::Matrix<double, 3, 1> E_2;
        Eigen::Matrix<double, 3, 1> E_3;
        Eigen::Matrix<double, 3, 3> Initial_Matrix;
        Eigen::Matrix<double, 3, 3> Identity_Matrix;
        double length;
        double alpha_yaw;
        double alpha_pitch;
        double bias_yaw = 0;
        double bias_pitch = 0;
        double s_vel = 1;

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

        double s = 0;

        void SolveDE(){
            K_a_c = s_vel * Func_c(s, E_1);
            K_a_1 = s_vel * Func_1(s, E_2, E_3);
            K_a_2 = s_vel * Func_2(s, E_1, E_3);
            K_a_3 = s_vel * Func_3(s, E_1, E_2);

            K_b_c = s_vel * Func_c(s + s_vel / 2, E_1 + K_a_1 / 2);
            K_b_1 = s_vel * Func_1(s + s_vel / 2, E_2 + K_a_2 / 2, E_3 + K_a_3 / 2);
            K_b_2 = s_vel * Func_2(s + s_vel / 2, E_1 + K_a_1 / 2, E_3 + K_a_3 / 2);
            K_b_3 = s_vel * Func_3(s + s_vel / 2, E_1 + K_a_1 / 2, E_2 + K_a_2 / 2);


            K_c_c = s_vel * Func_c(s + s_vel / 2, E_1 + K_b_1 / 2);
            K_c_1 = s_vel * Func_1(s + s_vel / 2, E_2 + K_b_2 / 2, E_3 + K_b_3 / 2);
            K_c_2 = s_vel * Func_2(s + s_vel / 2, E_1 + K_b_1 / 2, E_3 + K_b_3 / 2);
            K_c_3 = s_vel * Func_3(s + s_vel / 2, E_1 + K_a_1 / 2, E_2 + K_b_2 / 2);

            K_d_c = s_vel * Func_c(s + s_vel, E_1 + K_c_1);
            K_d_1 = s_vel * Func_1(s + s_vel, E_2 + K_c_2, E_3 + K_c_3);
            K_d_2 = s_vel * Func_2(s + s_vel, E_1 + K_c_1, E_3 + K_c_3);
            K_d_3 = s_vel * Func_3(s + s_vel, E_1 + K_c_1, E_2 + K_c_2);

            s+=s_vel;
            C   += (K_a_c + 2 * K_b_c + 2 * K_c_c + K_d_c) / 6;
            E_1 += (K_a_1 + 2 * K_b_1 + 2 * K_c_1 + K_d_1) / 6;
            E_2 += (K_a_2 + 2 * K_b_2 + 2 * K_c_2 + K_d_2) / 6;
            E_3 += (K_a_3 + 2 * K_b_3 + 2 * K_c_3 + K_d_3) / 6;
        
            C_x.push_back(C(0, 0));
            C_y.push_back(C(1, 0));
            C_z.push_back(C(2, 0));
        }
        double curvature_yaw(double _s){
            //横うねり推進(岡山大学論文参照)
            //std::cout<<"alphayaw:"<<alpha_yaw*180/M_PI<<std::endl;
            return alpha_yaw*M_PI*sin(2*M_PI*_s+M_PI/(2*length))/(2* length)+bias_yaw;
        }
        double curvature_pitch(double _s){
            return alpha_pitch*M_PI*sin(4*M_PI*_s+M_PI/(2*length))/(2* length)+bias_pitch;
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
};
