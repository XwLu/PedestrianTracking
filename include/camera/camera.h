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
        Eigen::Vector3d ProjectivePixel2Camera(Eigen::Vector2d _uv, double _depth);

    private:
        double cx_, cy_, fx_, fy_;
        Eigen::Matrix3d K_;
    };
}
#endif //PEDESTRIAN_TRACKING_CAMERA_H
