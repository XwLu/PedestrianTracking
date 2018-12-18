//
// Created by luyifan on 18-12-18.
//

#ifndef PEDESTRIAN_TRACKING_UTILS_H
#define PEDESTRIAN_TRACKING_UTILS_H

#include <Eigen/Core>

namespace luyifan{
    inline Eigen::Vector2i Average(std::vector<Eigen::Vector2i> _As){
        double sum1 = 0, sum2 = 0;
        for(auto A : _As){
            sum1 += A(0);
            sum2 += A(1);
        }
        return Eigen::Vector2i(sum1/_As.size(), sum2/_As.size());
    }
}

#endif //PEDESTRIAN_TRACKING_UTILS_H
