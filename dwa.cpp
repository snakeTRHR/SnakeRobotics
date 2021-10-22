#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>
#include<string>
#include<array>
#include"matplotlibcpp.h"

namespace plt = matplotlibcpp;

struct GoalSet{
    GoalSet(double _x, double _y):x(_x), y(_y){
    };
    double x;
    double y;
};

struct PathSet{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> th;
    double u_v;
    double u_th;
};

struct NextPathSet{
    std::vector<double> next_x;
    std::vector<double> next_y;
    std::vector<double> next_th;
};

struct Range{
    double min_ang_velo;
    double max_ang_velo;
    double min_velo;
    double max_velo;
};

struct ObsSet{
    ObsSet(){
    }
    ObsSet(double _x, double _y, double _r):x(_x), y(_y), r(_r){
        flag = false;
    }
    double x;
    double y;
    double r;
    bool flag;
};

class DWA{
    public:
        DWA(GoalSet _goal){
            g_x = _goal.x;
            g_y = _goal.y;
            TwoWheelRobot(0, 0, 0);
        }
        std::vector<double> plot_x;
        std::vector<double> plot_y;
        void Animation(double _x, double _y, std::vector<ObsSet> _obsPos){
            plot_x.push_back(_x);
            plot_y.push_back(_y);
            plt::clf();
            plt::xlim(0, 12);
            plt::ylim(0, 12);
            plt::plot(plot_x, plot_y);
            
            std::vector<PathSet> paths = traj_paths.back();
            for(int i = 0; i < paths.size(); ++i){
                PathSet tempPath = paths[i];
                std::vector<double> temp_path_x = tempPath.x;
                std::vector<double> temp_path_y = tempPath.y;
                plt::plot(temp_path_x, temp_path_y, "y");
            }
            //write_area
            /*double area_radius = 5.0;
            std::vector<double> area_write_x;
            std::vector<double> area_write_y;
            for(int k = 0; k < 100; ++k){
                double area_theta = 2 * M_PI * k / 100;
                area_write_x.push_back(robot_x + area_radius * cos(area_theta));
                area_write_y.push_back(robot_y + area_radius * sin(area_theta));
            }
            plt::plot(area_write_x, area_write_y, "r");
            */
            //write circle
            for(int i = 0; i < _obsPos.size(); ++i){
                ObsSet tempObs = _obsPos[i];
                double center_x = tempObs.x;
                double center_y = tempObs.y;
                double radius = tempObs.r;
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
            PathSet tempOpt = traj_opt.back();
            std::vector<double> temp_x = tempOpt.x;
            std::vector<double> temp_y = tempOpt.y;
            plt::plot(temp_x, temp_y, "g");
            
            plt::named_plot("DWA", plot_x, plot_y);
            plt::legend();
            plt::pause(0.01);
        }
        void runToGoal(GoalSet _goal, std::vector<ObsSet> _obs_pos){
            g_x = _goal.x;
            g_y = _goal.y;
            double time_step = 0;
            obs = _obs_pos;

            PathSet optPath = calcInput();

            double opt_u_v = optPath.u_v;
            double opt_u_th = optPath.u_th;
            robotUpdateState(opt_u_th, opt_u_v, sampling_time);
            time_step += 1;
            Animation(robot_x, robot_y, obs);
        }
        bool goalCheck(){
            double dis_to_goal = std::sqrt(std::pow((g_x - robot_x), 2) + std::pow((g_y - robot_y), 2));
            if(dis_to_goal < 0.5){
                return true;
            }else{
                return false;
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
        const double weight_angle = 0.05;
        const double weight_velo = 0.2;
        const double weight_obs = 0.1;

        std::vector<std::vector<PathSet>> traj_paths;
        std::vector<PathSet> traj_opt;

        double g_x;
        double g_y;

        std::vector<ObsSet> obs;

        PathSet calcInput(){
            //path作成
            std::vector<PathSet> paths = makePath();
            //path評価
            PathSet opt_path = evalPath(paths);
            traj_opt.push_back(opt_path);
            return opt_path;
        }
        Range calcRangeVelos(){
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
            Range min_max_range;
            min_max_range.min_ang_velo = min_ang_velo;
            min_max_range.max_ang_velo = max_ang_velo;
            min_max_range.min_velo = min_velo;
            min_max_range.max_velo = max_velo;
            return min_max_range;
        }
        NextPathSet predictState(double _ang_velo, double _velo, double _x, double _y, double _th, double _dt, double _pre_step){
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
            NextPathSet nextSet;
            nextSet.next_x = next_xs;
            nextSet.next_y = next_ys;
            nextSet.next_th = next_ths;
            return nextSet;
        }
        std::vector<PathSet> makePath(){
            //角度と速度の組み合わせを全探索
            std::vector<PathSet> paths;
            Range minMaxVal = calcRangeVelos();
            double min_ang_velo = minMaxVal.min_ang_velo;
            double max_ang_velo = minMaxVal.max_ang_velo;
            double min_velo = minMaxVal.min_velo;
            double max_velo = minMaxVal.max_velo;

            for(double ang_velo = min_ang_velo; ang_velo < max_ang_velo; ang_velo += delta_ang_velo){
                for(double velo = min_velo; velo < max_velo; velo += delta_velo){
                    double temp_x, temp_y, temp_th;
                    NextPathSet nextPath = predictState(ang_velo, velo, robot_x, robot_y, robot_th, sampling_time, pre_step);
                    PathSet path;
                    path.x = nextPath.next_x;
                    path.y = nextPath.next_y;
                    path.th = nextPath.next_th;
                    path.u_v = velo;
                    path.u_th = ang_velo;
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
        double headingAngle(PathSet _path, double _g_x, double _g_y){
            std::vector<double> temp_x = _path.x;
            std::vector<double> temp_y = _path.y;
            std::vector<double> temp_th = _path.th;
            double last_x = temp_x.back();
            double last_y = temp_y.back();
            double last_th = temp_th.back();

            //角度計算
            double angle_to_goal = atan2((_g_y - last_y), (_g_x - last_x));
            double score_angle = angle_to_goal - last_th;

            //ぐるぐる防止
            score_angle = abs(angleRangeCorrector(score_angle));

            //最大と最小をひっくり返す
            score_angle = M_PI - score_angle;
            return score_angle;
        }
        double headingVelo(PathSet _path){
            double temp_path_u_v = _path.u_v;
            double score_heading_velo = temp_path_u_v;
            return score_heading_velo;
        }
        std::vector<ObsSet> calcNearestObs(std::vector<ObsSet> _obstacles){
            std::vector<ObsSet> nearestObs;
            double area_dis_to_obs = 5;
            //double area_dis_to_obs = 2;
            for(int i = 0; i < _obstacles.size(); ++i){
                ObsSet tempObs = _obstacles[i];
                double temp_dis_to_obs = std::sqrt(std::pow(robot_x - tempObs.x, 2) + std::pow(robot_y - tempObs.y, 2));
                if(temp_dis_to_obs < area_dis_to_obs){
                    nearestObs.push_back(obs[i]);
                }
            }
            return nearestObs;
        }
        double obstacleCheck(PathSet _path, std::vector<ObsSet> _nearestObs){
            double score_obstacle = 0.0;
            double temp_dis_to_obs = 0.0;
            
            std::vector<double> temp_path_x = _path.x;
            std::vector<double> temp_path_y = _path.y;
            for(int i = 0; i < temp_path_x.size(); ++i){
                for(int k = 0; k < _nearestObs.size(); ++k){
                    ObsSet temp_obs = _nearestObs[k];
                    temp_dis_to_obs = std::sqrt(std::pow((temp_path_x[i] - temp_obs.x), 2) + std::pow((temp_path_y[i] - temp_obs.y), 2));
                    
                    if(temp_dis_to_obs < score_obstacle){
                        score_obstacle = temp_dis_to_obs;
                    }
                    if(temp_dis_to_obs < temp_obs.r + 0.75){
                        score_obstacle = -INFINITY;
                    }
                }
            }
            return score_obstacle;
            /*if(score_obstacle < 1){
                return -INFINITY;
            }else{
                return score_obstacle;
            }*/
        }
        std::vector<double> minMaxNormalize(std::vector<double> val){
            double max_data = *std::max_element(val.begin(), val.end());
            double min_data = *std::min_element(val.begin(), val.end());
            if((max_data - min_data) == 0){
                return std::vector<double>(val.size(), 0);
            }else{
                for(int i = 0; i < val.size(); ++i){
                    val[i] = (val[i] - min_data) / (max_data - min_data);
                }
                return val;
            }
        }
        PathSet evalPath(std::vector<PathSet> _paths){
            std::vector<ObsSet> nearestObs = calcNearestObs(obs);
            
            std::vector<double> score_heading_angles;
            std::vector<double> score_heading_velos;
            std::vector<double> score_obstacles;

            for(int i = 0; i < _paths.size(); ++i){
                score_heading_angles.push_back(headingAngle(_paths[i], g_x, g_y));
                score_heading_velos.push_back(headingVelo(_paths[i]));
                score_obstacles.push_back(obstacleCheck(_paths[i], nearestObs));
            }
            //正規化
            minMaxNormalize(score_heading_angles);
            minMaxNormalize(score_heading_velos);
            minMaxNormalize(score_obstacles);
            double score = 0.0;
            double temp_score = 0.0;
            
            PathSet optPath;
            for(int i = 0; i < _paths.size(); ++i){
                temp_score = weight_angle * score_heading_angles[i] +
                             weight_velo * score_heading_velos[i] +
                             weight_obs * score_obstacles[i];
                //std::cout << score_heading_angles[i] << " " << score_heading_velos[i] << " " << score_obstacles[i] << std::endl;
                if(temp_score > score){
                    optPath = _paths[i];
                    score = temp_score;
                }
            }
            //std::cout << "score " << score << std::endl;
            return optPath;
        }
};

ObsSet changeObsPos(ObsSet _prevObs){
    ObsSet nextObs;
    nextObs.x = _prevObs.x;
    nextObs.r = _prevObs.r;
    if(_prevObs.flag == false){
        nextObs.y = _prevObs.y + 0.1;
    }else if(_prevObs.flag == true){
        nextObs.y = _prevObs.y -0.1;
    }
    if(nextObs.y > 11){
        nextObs.flag = true;
    }else if(nextObs.y < -11){
        nextObs.flag = false;
    }else{
        nextObs.flag = _prevObs.flag;
    }
    return nextObs;
}

int main(){
    std::vector<ObsSet> obsPos;
    ObsSet obs[6]{ObsSet(7.5, -1.0, 0.25),
                  ObsSet(8.5, 3.0, 0.25),
                  ObsSet(5.0, 6.5, 0.25),
                  ObsSet(11.0, -1.5, 0.25),
                  ObsSet(9.0, -4.5, 0.25),
                  ObsSet(4.0, 3.0, 0.25)};
    /*ObsSet obs[6]{ObsSet(7.5, 1.0, 0.25),
                  ObsSet(7.5, 4.0, 0.25),
                  ObsSet(5.0, 12.5, 0.25),
                  ObsSet(11.0, -1.5, 0.25),
                  ObsSet(9.0, -4.5, 0.25),
                  ObsSet(4.0, 8.0, 0.25)};
    */
    for(int i = 0; i < 6; ++i){
        obsPos.push_back(obs[i]);
    }

    GoalSet goalPos(10, 10);

    DWA dwa(goalPos);
    bool finish = false;
    while(finish == false){
        obsPos.clear();
        for(int i = 0; i < 6; ++i){
            obs[i] = changeObsPos(obs[i]);
            obsPos.push_back(obs[i]);
        }
        dwa.runToGoal(goalPos, obsPos);
        finish = dwa.goalCheck();
    }
}