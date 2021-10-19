#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>
#include<string>
#include"matplotlibcpp.h"

namespace plt = matplotlibcpp;
//x, y, th, u_v, u_th
using path_tuple = std::tuple<double, double, double, double, double>;
//x, y, size
using obs_tuple = std::tuple<double, double, double>;
//x, y
using info_tuple = std::tuple<double, double>

//myenigma参考
class DWA{
    public:
        DWA(){

        }
        void update(std::vector<obs_tuple> _obs_pos){

        }
        info_tuple getInfo(){
            return info_tuple(std::get<0>(opt_path), std::get<1>opt_path);
        }
    private:
        //パラメータ
        double max_ang_accelation;
        double max_accelation;
        double range_ang_velo;
        double lim_min_ang_velo;
        double lim_max_ang_velo;
        double lim_min_velo;
        double lim_max_velo;

        //予測時間(s)
        double pre_time;
        double pre_step;
        //探索時の刻み幅
        double delta_velo;
        double delta_ang_velo;
        //サンプリングタイム
        double sampling_time;
        //重み付け
        double weight_angle;
        double weight_velo;
        double weight_obs;
        std::vector<double> traj_paths;
        std::vector<double> traj_opt;

        //現在の値
        double velo_now;
        double ang_velo_now;

        //あとで書き換える
        double min_ang_velo;
        double max_ang_velo;
        double min_velo;
        double max_velo;
        std::vector<double> next_x;
        std::vector<double> next_y;
        std::vector<double> next_th;

        path_tuple opt_path;
        std::vector<path_tuple> path;
        std::vector<std::vector<path_tuple>> traj_path;

        double x;
        double y;
        double th;

        double g_x;
        double g_y;

        std::vector<obs_tuple> obs;
        std::vector<obs_tuple> nearest_obs;

        void calcRangeVelos(){
            //角速度
            double range_ang_velo = sampling_time * max_ang_accelation;
            min_ang_velo = ang_velo_now - range_ang_velo;
            max_ang_velo = ang_velo_now + range_ang_velo;
            //最小値
            if(min_ang_velo < lim_min_ang_velo){
                min_ang_velo = lim_min_ang_velo;
            }
            //最大値
            if(max_ang_velo > lim_min_ang_velo){
                max_ang_velo = lim_min_ang_velo;
            }

            //速度            
            double range_velo = sampling_time * max_accelation;
            min_velo = velo_now - range_velo;
            max_velo = velo_now + range_velo;
            //最小値
            if(min_velo < lim_min_velo){
                min_velo = lim_min_velo;
            }
            //最大値
            if(max_velo > lim_max_velo){
                max_velo = lim_max_velo;
            }
            
        }
        void predictState(double _ang_velo, double _velo, double _x, double _y, double _th, double _dt, double _pre_step){
            //予想状態を作成する
            double temp_x, temp_y, temp_th;
            for(int i = 0; i < _pre_step; ++i){
                temp_x = _velo * cos(_th) * _dt + _x;
                temp_y = _velo * sin(_th) * _dt + _y;
                temp_th = _ang_velo * _dt + _th;

                next_x.push_back(temp_x);
                next_y.push_back(temp_y);
                next_th.push_back(temp_th);
            }
        }
        double makePath(){
            //角度と速度の組み合わせを全探索
            path.clear();
            for(double ang_velo = min_ang_velo; ang_velo < max_ang_velo; ang_velo += delta_ang_velo){
                for(double velo = min_velo; velo < max_velo; velo += delta_velo){
                    double temp_x, temp_y, temp_th;
                    predictState(ang_velo, velo, x, y, th, sampling_time, pre_step);
                    path.push_back(path_tuple(temp_x, temp_y, temp_th, ang_velo, velo));
                }
            }
            //時刻歴Pathを保存
            traj_path.push_back(path);
        }
        double angleRangeCorrector(double _angle){
            if(_angle > M_PI){
                while(_angle > M_PI){
                    _angle -= 2 * M_PI;
                }
            }else if(_angle < -M_PI){
                while(_angle < -M_PI){
                    _angle += 2 * M_PI;
                }
            }
            return _angle;
        }
        double headingAngle(path_tuple _path, double _g_x, double _g_y){
            double last_x = std::get<0>(_path);
            double last_y = std::get<1>(_path);
            double last_th = std::get<2>(_path);

            //角度計算
            double angle_to_goal = atan2(_g_y - last_y, _g_x - last_x);

            //score計算
            double score_angle = angle_to_goal - last_th;

            //ぐるぐる防止
            score_angle = abs(angleRangeCorrector(score_angle));

            //最大と最小をひっくり返す
            score_angle = M_PI - score_angle;

            return score_angle;
        }
        double headingVelo(path_tuple _path){
            double score_heading_velo = std::get<3>(_path);
            return score_heading_velo;
        }
        void calcNearestObs(){
            nearest_obs.clear();
            double area_dis_to_obs = 5;
            for(int i = 0; i < obs.size(); ++i){
                obs_tuple temp_obs = obs[i];
                double temp_dis_to_obs = std::sqrt(std::pow(x - std::get<0>(temp_obs), 2) + std::pow(y - std::get<1>(temp_obs), 2));
                if(temp_dis_to_obs < area_dis_to_obs){
                    nearest_obs.push_back(obs[i]);
                }
            }

        }
        double obstacleCheck(path_tuple _path){
            double score_obstacle = 2;
            double temp_dis_to_obs = 0.0;
            
            //違う
            for(int i = 0; i < std::get<0>(_path), ++i){
                for(int k = 0; k < nearest_obs.size(); ++k){
                    obs_tuple temp_obs = nearest_obs[k];
                    temp_dis_to_obs = std::sqrt(std::pow(std::get<0>(_path) - std::get<0>(temp_obs), 2) + std::pow(std::get<1>(_path) - std::get<1>(temp_obs), 2));
                    
                    if(temp_dis_to_obs < score_obstacle){
                        score_obstacle = temp_dis_to_obs;
                    }
                    if(temp_dis_to_obs < std::get<2>(temp_obs) + 0.75){
                        score_obstacle = -999999;
                    }
                }
            }
            return score_obstacle;
        }
        void minMaxNormalize(){

        }
        void evalPath(){
            std::vector<double> score_heading_angles;
            std::vector<double> score_heading_velos;
            std::vector<double> score_obstacles;

            for(int i = 0; i < path.size(); ++i){
                score_heading_angles.push_back(headingAngle(path[i], g_x, g_y));
                score_heading_velos.push_back(headingVelo(path[i]));
                score_obstacles.push_back(obstacleCheck());
            }
            //正規化
            for(int i = 0; i < path.size(); ++i){
                minMaxNormalize(score_heading_angles[i], score_heading_velos[i], score_obstacles[i]);
            }
            double score = 0.0;
            //最小pathを探索
            for(int i = 0; i < path.size(); ++i){
                double temp_score = 0.0;
                temp_score = weight_angle * score_heading_angles[i] +
                             weight_velo * score_heading_velos[i] +
                             weight_obs * score_obstacles[i];
                if(temp_score > score){
                    opt_path = path[i];
                    score = temp_score;
                }

            }
        }
};

class TwoWheelRobot(){
    public:
        TwoWheelRobot(double _init_x, double _init_y, double _init_th){
            x = _init_x;
            y = _init_y;
            th = _init_th;
        }
    private:
        double x;
        double y;
        double th;
}

void Animation(double _x, double _y){
    plt::clf();
    plt::plot(_x, _y);
    plt::legend();
    plt::pause(0.01);
}
int main(){
    std::vector<obs_tuple> obsPos;
    obsPos.push_back(obs_tuple(1, 1, 0.5));
    obsPos.push_back(obs_tuple(2, 2, 0.5));
    obsPos.push_back(obs_tuple(2, 1, 0.5));

    DWA dwa;
    while(1){
        info_tuple temp_info;
        dwa.update(obsPos);
        temp_info = dwa.getInfo();
        
        double pos_x = std::get<0>temp_info;
        double pos_y = std::get<1>temp_info;
        
        Animation(pos_x, pos_y);
    }
}