//
// Created by luyifan on 18-12-17.
//

#ifndef PEDESTRIAN_TRACKING_GUASSN_H
#define PEDESTRIAN_TRACKING_GUASSN_H

#include <cmath>

namespace luyifan{

#define PI 3.1415926

class Gaussian{
public:
    Gaussian(double _e1, double _e2, double _v1, double _v2);
    ~Gaussian();

    inline double GetValue(double _x, double _y){
        return pow(2*PI*variance1_*variance2_, -1.0)*exp(-0.5 * (pow(_x-expect1_,2.0)/variance1_2_ + pow(_y-expect2_,2.0)/variance2_2_));
    }
private:
    double expect1_;
    double expect2_;
    double variance1_;
    double variance1_2_;
    double variance2_;
    double variance2_2_;
};
}

#endif //PEDESTRIAN_TRACKING_GUASSN_H
