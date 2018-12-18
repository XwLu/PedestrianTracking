//
// Created by luyifan on 18-12-18.
//

#ifndef PEDESTRIAN_TRACKING_PARTICLEFILTER_H
#define PEDESTRIAN_TRACKING_PARTICLEFILTER_H

#include "opencv2/opencv.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include <Eigen/Core>

namespace luyifan{
    class ParticleFilter{
    public:
        ParticleFilter(int _num, std::vector<geometry_msgs::PointStamped> _candidates);
        ~ParticleFilter();

        inline std::vector<geometry_msgs::Pose> Particles() {return particles_;}
        inline geometry_msgs::Pose Result(){return result_;}
        void Update(std::vector<geometry_msgs::PointStamped> _candidates);
        std::vector<std::pair<int, Eigen::Vector2d>> DistributeParticles(std::vector<geometry_msgs::PointStamped> _candidates);
        void ParticlesInit(std::vector<geometry_msgs::PointStamped> _candidates);
        void ScatterParticles(std::vector<std::pair<int, Eigen::Vector2d>> _distribution);

    private:
        std::vector<geometry_msgs::Pose> particles_;
        int num_;
        geometry_msgs::Pose result_;
        geometry_msgs::Twist vel_;
    };
};

#endif //PEDESTRIAN_TRACKING_PARTICLEFILTER_H
