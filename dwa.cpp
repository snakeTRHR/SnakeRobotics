#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>
#include<string>
#include"matplotlibcpp.h"

namespace plt = matplotlibcpp;
//x, y, th, u_v, u_th
using path_tuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, double, double>;
//x, y, size
using obs_tuple = std::tuple<double, double, double>;
//x, y
using info_tuple = std::tuple<double, double, double>;
//
using next_tuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;
//min_ang_velo, max_ang_velo, min_velo, max_velo
using range_tuple = std::tuple<double, double, double, double>;
//myenigma参考
class DWA{
    public:
        DWA(double _g_x, double _g_y){
            g_x = _g_x;
            g_y = _g_y;
            TwoWheelRobot(0, 0, 0);
        }
        std::vector<double> plot_x;
        std::vector<double> plot_y;
        void Animation(double _x, double _y, std::vector<obs_tuple> _obs_pos){
            plot_x.push_back(_x);
            plot_y.push_back(_y);
            plt::clf();
            plt::xlim(-12, 12);
            plt::ylim(-12, 12);
            plt::plot(plot_x, plot_y);
            
            std::vector<path_tuple> paths = traj_paths.back();
            for(int i = 0; i < paths.size(); ++i){
                path_tuple temp_path = paths[i];
                std::vector<double> temp_path_x = std::get<0>(temp_path);
                std::vector<double> temp_path_y = std::get<1>(temp_path);
                plt::plot(temp_path_x, temp_path_y, "y");
            }
            //write_area
            double area_radius = 5.0;
            std::vector<double> area_write_x;
            std::vector<double> area_write_y;
            for(int k = 0; k < 100; ++k){
                double area_theta = 2 * M_PI * k / 100;
                area_write_x.push_back(robot_x + area_radius * cos(area_theta));
                area_write_y.push_back(robot_y + area_radius * sin(area_theta));
            }
            plt::plot(area_write_x, area_write_y, "r");
            
            //write circle
            for(int i = 0; i < _obs_pos.size(); ++i){
                obs_tuple temp_obs = _obs_pos[i];
                double center_x = std::get<0>(temp_obs);
                double center_y = std::get<1>(temp_obs);
                double radius = std::get<2>(temp_obs);
                std::vector<double> write_x;
                std::vector<double> write_y;
                for(int k = 0; k < 100; ++k){
                    double theta = 2 * M_PI * k / 100;
                    write_x.push_back(center_x + radius * cos(theta));
                    write_y.push_back(center_y + radius * sin(theta));
                }
                plt::plot(write_x, write_y, "b");
            }
            //write goal
            double goal_radius = 0.5;
            std::vector<double> goal_write_x;
            std::vector<double> goal_write_y;
            for(int k = 0; k < 100; ++k){
                double goal_theta = 2 * M_PI * k / 100;
                goal_write_x.push_back(g_x + goal_radius * cos(goal_theta));
                goal_write_y.push_back(g_y + goal_radius * sin(goal_theta));
            }
            plt::plot(goal_write_x, goal_write_y, "r");
           
            //write opt path
            path_tuple temp_opt = traj_opt.back();
            std::vector<double> temp_x = std::get<0>(temp_opt);
            std::vector<double> temp_y = std::get<1>(temp_opt);
            plt::plot(temp_x, temp_y, "g");
            
            plt::named_plot("DWA", plot_x, plot_y);
            plt::legend();
            plt::pause(0.01);
        }
        void runToGoal(std::vector<obs_tuple> _obs_pos){
            bool goal_flag = false;
            double time_step = 0;
            while(goal_flag == false){
                obs = _obs_pos;

                path_tuple opt_path = calcInput();

                double u_v = std::get<3>(opt_path);
                double u_th = std::get<4>(opt_path);
                robotUpdateState(u_th, u_v, sampling_time);
                double dis_to_goal = std::sqrt(std::pow((g_x - robot_x), 2) + std::pow((g_y - robot_y), 2));
                if(dis_to_goal < 0.5){
                    goal_flag = true;
                }
                time_step += 1;
                Animation(robot_x, robot_y, obs);
                //std::cout << robot_x << " " << robot_y << std::endl;
            }            
        }
        void TwoWheelRobot(double _init_x, double _init_y, double _init_th){
            robot_x = _init_x;
            robot_y = _init_y;
            robot_th = _init_th;
            robot_u_v = 0.0;
            robot_u_th = 0.0;
            robot_traj_x.push_back(robot_x);
            robot_traj_y.push_back(robot_y);
            robot_traj_th.push_back(robot_th);
            robot_traj_u_v.push_back(robot_u_v);
            robot_traj_u_th.push_back(robot_u_th);
        }
        void robotUpdateState(double _u_th, double _u_v, double _dt){
            robot_u_th = _u_th;
            robot_u_v = _u_v;

            double robot_next_x = robot_u_v * cos(robot_th) * _dt + robot_x;
            double robot_next_y = robot_u_v * sin(robot_th) * _dt + robot_y;
            double robot_next_th = robot_u_th * _dt + robot_th;

            robot_traj_x.push_back(robot_next_x);
            robot_traj_y.push_back(robot_next_y);
            robot_traj_th.push_back(robot_next_th);

            robot_x = robot_next_x;
            robot_y = robot_next_y;
            robot_th = robot_next_th;        
        }
        double robot_x;
        double robot_y;
        double robot_th;
        double robot_u_v;
        double robot_u_th;
        std::vector<double> robot_traj_x;
        std::vector<double> robot_traj_y;
        std::vector<double> robot_traj_th;
        std::vector<double> robot_traj_u_v;
        std::vector<double> robot_traj_u_th;
    private:
        //パラメータ
        const double max_ang_accelation = 100 * M_PI / 180;
        const double max_accelation = 1.0;
        const double lim_min_ang_velo = -M_PI;
        const double lim_max_ang_velo = M_PI;
        const double lim_min_velo = 0.0;
        const double lim_max_velo = 1.6;

        //予測時間(s)
        const double pre_time = 3;
        const double pre_step = 30;
        //探索時の刻み幅
        const double delta_velo = 0.02;
        const double delta_ang_velo = 0.02;
        //サンプリングタイム
        const double sampling_time = 0.1;
        //重み付け
        const double weight_angle = 500.0;
        const double weight_velo = 0.2;
        const double weight_obs = 0.01;

        std::vector<std::vector<path_tuple>> traj_paths;
        std::vector<path_tuple> traj_opt;

        double g_x;
        double g_y;

        std::vector<obs_tuple> obs;

        path_tuple calcInput(){
            //path作成
            std::vector<path_tuple> paths = makePath();
            //path評価
            path_tuple opt_path = evalPath(paths);
            traj_opt.push_back(opt_path);
            return opt_path;
        }
        range_tuple calcRangeVelos(){
            //角速度
            double range_ang_velo = sampling_time * max_ang_accelation;
            double min_ang_velo = robot_u_th - range_ang_velo;
            double max_ang_velo = robot_u_th + range_ang_velo;


            //最小値
            if(min_ang_velo < lim_min_ang_velo){
                min_ang_velo = lim_min_ang_velo;
            }
            //最大値
            if(max_ang_velo > lim_max_ang_velo){
                max_ang_velo = lim_max_ang_velo;
            }

            //速度            
            double range_velo = sampling_time * max_accelation;
            double min_velo = robot_u_v - range_velo;
            double max_velo = robot_u_v + range_velo;
            //最小値
            if(min_velo < lim_min_velo){
                min_velo = lim_min_velo;
            }
            //最大値
            if(max_velo > lim_max_velo){
                max_velo = lim_max_velo;
            }
            return range_tuple(min_ang_velo, max_ang_velo, min_velo, max_velo);
        }
        next_tuple predictState(double _ang_velo, double _velo, double _x, double _y, double _th, double _dt, double _pre_step){
            //予想状態を作成する
            std::vector<double> next_xs;
            std::vector<double> next_ys;
            std::vector<double> next_ths;

            double temp_x, temp_y, temp_th;

            for(int i = 0; i < _pre_step; ++i){
                temp_x = _velo * cos(_th) * _dt + _x;
                temp_y = _velo * sin(_th) * _dt + _y;
                temp_th = _ang_velo * _dt + _th;

                next_xs.push_back(temp_x);
                next_ys.push_back(temp_y);
                next_ths.push_back(temp_th);

                _x = temp_x;
                _y = temp_y;
                _th = temp_th;
            }
            return next_tuple(next_xs, next_ys, next_ths);
        }
        std::vector<path_tuple> makePath(){
            //角度と速度の組み合わせを全探索
            std::vector<path_tuple> paths;
            range_tuple min_max_val = calcRangeVelos();
            double min_ang_velo = std::get<0>(min_max_val);
            double max_ang_velo = std::get<1>(min_max_val);
            double min_velo = std::get<2>(min_max_val);
            double max_velo = std::get<3>(min_max_val);

            for(double ang_velo = min_ang_velo; ang_velo < max_ang_velo; ang_velo += delta_ang_velo){
                for(double velo = min_velo; velo < max_velo; velo += delta_velo){
                    double temp_x, temp_y, temp_th;
                    next_tuple next_path = predictState(ang_velo, velo, robot_x, robot_y, robot_th, sampling_time, pre_step);
                    path_tuple path;
                    path = path_tuple(std::get<0>(next_path), std::get<1>(next_path), std::get<2>(next_path), velo, ang_velo);
                    paths.push_back(path);
                }
            }
            traj_paths.push_back(paths);
            return paths;
        }
        double angleRangeCorrector(double _angle){
            if(_angle > M_PI){
                while(_angle > M_PI){
                    _angle -= (2 * M_PI);
                }
            }else if(_angle < -M_PI){
                while(_angle < -M_PI){
                    _angle += (2 * M_PI);
                }
            }
            return _angle;
        }
        double headingAngle(path_tuple _path, double _g_x, double _g_y){
            std::vector<double> temp_x = std::get<0>(_path);
            std::vector<double> temp_y = std::get<1>(_path);
            std::vector<double> temp_th = std::get<2>(_path);
            double last_x = temp_x.back();
            double last_y = temp_y.back();
            double last_th = temp_th.back();

            //角度計算
            //std::cout << "robot_th " << robot_th * 180 / M_PI<< std::endl;
            std::cout << "th " << last_th * 180 / M_PI << std::endl;
            //std::cout << "val:" << _g_y << " " << last_y << " " << _g_x << " " << last_x << std::endl;
            double angle_to_goal = atan2((_g_y - last_y), (_g_x - last_x));
            //double angle_to_goal = atan2((_g_x - last_x), (_g_y - last_y));
            std::cout << "goal " << angle_to_goal * 180.0 / M_PI << std::endl;
            //score計算
            double score_angle = angle_to_goal - last_th;
            //std::cout << "score " << score_angle << std::endl;

            //ぐるぐる防止
            score_angle = abs(angleRangeCorrector(score_angle));

            //最大と最小をひっくり返す
            score_angle = M_PI - score_angle;
            std::cout << "score " << score_angle * 180 / M_PI << std::endl;
            return score_angle;
        }
        double headingVelo(path_tuple _path){
            double temp_path_u_v = std::get<3>(_path);
            double score_heading_velo = temp_path_u_v;
            //std::cout << "heading velo:" << score_heading_velo << std::endl;
            return score_heading_velo;
        }
        std::vector<obs_tuple> calcNearestObs(std::vector<obs_tuple> _obstacles){
            std::vector<obs_tuple> nearest_obs;
            //double area_dis_to_obs = 5;
            double area_dis_to_obs = 2;
            for(int i = 0; i < _obstacles.size(); ++i){
                obs_tuple temp_obs = _obstacles[i];
                double temp_dis_to_obs = std::sqrt(std::pow(robot_x - std::get<0>(temp_obs), 2) + std::pow(robot_y - std::get<1>(temp_obs), 2));
                if(temp_dis_to_obs < area_dis_to_obs){
                    nearest_obs.push_back(obs[i]);
                }
            }
            return nearest_obs;
        }
        double obstacleCheck(path_tuple _path, std::vector<obs_tuple> _nearest_obs){
            double score_obstacle = 0.0;
            double temp_dis_to_obs = 0.0;
            
            std::vector<double> temp_path_x = std::get<0>(_path);
            std::vector<double> temp_path_y = std::get<1>(_path);
            for(int i = 0; i < temp_path_x.size(); ++i){
                for(int k = 0; k < _nearest_obs.size(); ++k){
                    obs_tuple temp_obs = _nearest_obs[k];
                    temp_dis_to_obs = std::sqrt(std::pow((temp_path_x[i] - std::get<0>(temp_obs)), 2) + std::pow((temp_path_y[i] - std::get<1>(temp_obs)), 2));
                    
                    if(temp_dis_to_obs < score_obstacle){
                        score_obstacle = temp_dis_to_obs;
                    }
                    if(temp_dis_to_obs < std::get<2>(temp_obs) + 0.75){
                        score_obstacle = -INFINITY;
                    }
                    std::cout << "999999" << std::endl;
                }
            }
            return score_obstacle;
            /*if(score_obstacle < 1){
                return -INFINITY;
            }else{
                return score_obstacle;
            }*/
        }
        void minMaxNormalize(double &_angle, double &_velo, double &_obs){
            double max_data = std::max({_angle, _velo, _obs});
            double min_data = std::min({_angle, _velo, _obs});
            if((max_data - min_data) == 0){
                _angle = 0;
                _velo = 0;
                _obs = 0;
            }else{
                _angle = (_angle - min_data) / (max_data - min_data);
                _velo = (_velo - min_data) / (max_data - min_data);
                _obs = (_obs - min_data) / (max_data - min_data);
            }
        }
        path_tuple evalPath(std::vector<path_tuple> _paths){
            std::vector<obs_tuple> nearest_obs = calcNearestObs(obs);
            
            std::vector<double> score_heading_angles;
            std::vector<double> score_heading_velos;
            std::vector<double> score_obstacles;

            for(int i = 0; i < _paths.size(); ++i){
                score_heading_angles.push_back(headingAngle(_paths[i], g_x, g_y));
                score_heading_velos.push_back(headingVelo(_paths[i]));
                score_obstacles.push_back(obstacleCheck(_paths[i], nearest_obs));
            }
            //正規化
            for(int i = 0; i < _paths.size(); ++i){
                //std::cout << score_heading_angles[i] << " " << score_heading_velos[i] << " " << score_obstacles[i] << std::endl;
                minMaxNormalize(score_heading_angles[i], score_heading_velos[i], score_obstacles[i]);
                //std::cout << score_heading_angles[i] << " " << score_heading_velos[i] << " " << score_obstacles[i] << std::endl;
            }
            double score = 0.0;
            double temp_score = 0.0;
            
            path_tuple opt_path;
            for(int i = 0; i < _paths.size(); ++i){
                temp_score = weight_angle * score_heading_angles[i] +
                             weight_velo * score_heading_velos[i] +
                             weight_obs * score_obstacles[i];
                //std::cout << score_obstacles[i] << std::endl;
                if(temp_score > score){
                    opt_path = _paths[i];
                    score = temp_score;
                }
            }
            //std::cout << "score " << score << std::endl;
            return opt_path;
        }
};

int main(){
    std::vector<obs_tuple> obsPos;
    obsPos.push_back(obs_tuple(-4, 1, 0.25));
    obsPos.push_back(obs_tuple(0, 4.5, 0.25));
    obsPos.push_back(obs_tuple(3, 4.5, 0.25));
    obsPos.push_back(obs_tuple(5, 3.5, 0.25));
    obsPos.push_back(obs_tuple(7.5, 9.0, 0.25));
    obsPos.push_back(obs_tuple(6, 6, 0.25));
    obsPos.push_back(obs_tuple(7, 9.0, 0.25));
    obsPos.push_back(obs_tuple(10, 2.0, 0.25));
    obsPos.push_back(obs_tuple(10, 1.0, 0.25));
    obsPos.push_back(obs_tuple(10, 0.0, 0.25));
    obsPos.push_back(obs_tuple(11, -1.0, 0.25));
    double goal_x = 10;
    double goal_y = 10;

    DWA dwa(goal_x, goal_y);
    dwa.runToGoal(obsPos);
}