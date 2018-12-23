//
// Created by luyifan on 18-12-16.
//

#ifndef PEDESTRIAN_TRACKING_CAMERA_H
#define PEDESTRIAN_TRACKING_CAMERA_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include <memory>
#include <Eigen/Core>

namespace luyifan{
    class Camera{
    public:
        typedef std::shared_ptr<Camera> Ptr;

        Camera(double _cx, double _cy, double _fx, double _fy);
        ~Camera();
        Eigen::Vector3d ProjectivePixel2Camera(const Eigen::Vector2d& _uv, const double& _depth);
        Eigen::Vector2i ProjectiveCamera2Pixel(const Eigen::Vector3d& _pos);
        inline Eigen::Matrix3d K(){ return K_;}

    private:
        double cx_, cy_, fx_, fy_;
        Eigen::Matrix3d K_;
    };
}
#endif //PEDESTRIAN_TRACKING_CAMERA_H
