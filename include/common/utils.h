//
// Created by luyifan on 18-12-18.
//

#ifndef PEDESTRIAN_TRACKING_UTILS_H
#define PEDESTRIAN_TRACKING_UTILS_H

#include <Eigen/Core>
#include <iostream>
#include <fstream>

namespace luyifan{
    inline Eigen::Vector2i Average(const std::vector<Eigen::Vector2i>& _As){
        double sum1 = 0, sum2 = 0;
        for(auto A : _As){
            sum1 += A(0);
            sum2 += A(1);
        }
        return Eigen::Vector2i(sum1/_As.size(), sum2/_As.size());
    }

    inline double VelocityLimit(const double& _vel, const double& _limit){
        if(_vel > 0){
            return std::min(_limit, _vel);
        } else{
            return - std::min(_limit, -_vel);
        }
    }

    inline double Dist2Line(const Eigen::Vector2d& _begin, const Eigen::Vector2d& _end, const Eigen::Vector2d& _point){
        if(_begin(0) == _end(0)){
            return fabs(_point(0) - _begin(0));
        }

        double k = (_end(1) - _begin(1)) / (_end(0) - _begin(0));
        double b = _begin(1) - k * _begin(0);

        return fabs(k * _point(0) - _point(1) + b) / pow(1 + k*k, 0.5);
    }

    inline void WriteToText(const double& _x, const double& _y){
        std::ofstream mycout("/home/luyifan/tmp.txt");
        if(mycout){
            mycout<< _x<<" "<<_y<< std::endl;
            mycout.close();
        } else
            std::cerr<<"No txt to record data!"<<std::endl;
    }
}

#endif //PEDESTRIAN_TRACKING_UTILS_H
