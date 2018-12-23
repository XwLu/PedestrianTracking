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
#include <random>
#include "common/utils.h"

namespace luyifan{
    class ParticleFilter{
    public:
        ParticleFilter(int _num, std::vector<geometry_msgs::PointStamped>& _candidates);
        ~ParticleFilter();

        inline std::vector<geometry_msgs::Pose> Particles() {return particles_;}
        inline geometry_msgs::Pose Result(){return tmp_result_;}
        inline geometry_msgs::Pose TmpResult(){return tmp_result_;}
        void Update(const std::vector<geometry_msgs::PointStamped>& _candidates);
        std::vector<std::pair<int, Eigen::Vector2d>> DistributeParticles(const std::vector<geometry_msgs::PointStamped>& _candidates);
        void ParticlesInit(const std::vector<geometry_msgs::PointStamped>& _candidates);
        void ScatterParticles(const std::vector<std::pair<int, Eigen::Vector2d>>& _distribution);
        double EvaluateParticle(std::vector<geometry_msgs::PointStamped> _candidates, geometry_msgs::Pose _particle);
        void NormalizeConfidence(std::vector<geometry_msgs::Pose>& _particles, double& _sum_con);
        void Resampling(std::vector<geometry_msgs::Pose>& _particles, const double _sum_conf);
        geometry_msgs::Pose WeightedAverage(const std::vector<geometry_msgs::Pose>& _particles);
        inline double TimeNotInView(const ros::Time& _tNow){
            return _tNow.sec - last_update_.sec;
        }
    private:
        std::vector<geometry_msgs::Pose> particles_;
        int num_;
        geometry_msgs::Pose result_, tmp_result_;
        geometry_msgs::Twist vel_;

        double mean_;//均值
        double stddev_;//标准差
        std::default_random_engine generator_;
        std::normal_distribution<double> noise_;

        ros::Time last_update_;
    };
};

#endif //PEDESTRIAN_TRACKING_PARTICLEFILTER_H
