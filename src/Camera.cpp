//
// Created by luyifan on 18-12-16.
//

#include "camera/camera.h"

using namespace luyifan;
using namespace Eigen;

Camera::Camera(double _cx, double _cy, double _fx, double _fy):
        cx_(_cx), cy_(_cy), fx_(_fx), fy_(_fy){
    K_<<fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;

}

Camera::~Camera() {}

Vector3d Camera::ProjectivePixel2Camera(const Vector2d& _uv, const double& _depth) {
    return Vector3d (
         (_uv(0) - cx_)*_depth/fx_,
         (_uv(1) - cy_)*_depth/fy_,
         _depth
    );
}

Vector2i Camera::ProjectiveCamera2Pixel(const Eigen::Vector3d& _pos) {
    return Vector2i(
          fx_ * _pos(0) / _pos(2) + cx_,
          fy_ * _pos(1) / _pos(2) + cy_
    );
}