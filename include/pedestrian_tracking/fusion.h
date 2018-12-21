#ifndef PEDESTRIAN_TRACKING_FUSION_H
#define PEDESTRIAN_TRACKING_FUSION_H

#include "ros/ros.h"
#include "image_transport/subscriber.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "ros/callback_queue.h"
#include "camera/camera.h"
#include "geometry_msgs/PointStamped.h"
#include "common/gaussian.h"
#include "ParticleFilter.h"
#include "grid_map/GridMap.h"
#include "common/utils.h"
#include "std_msgs/Float64MultiArray.h"

namespace luyifan{

///Position of camera in grid map
#define CameraX 126
#define CameraY 108
//number of particles in PF
#define Partcile_NUM 20

    class Fusion{
    public:
        Fusion(Camera::Ptr _camera, GridMap::Ptr _map);
        ~Fusion();

        void Process(const cv::Mat& _map, const Eigen::Vector4i& _uvs);
        void Show(const cv::Mat& _map);
        Eigen::Vector2i ProjectiveCamera2GridPixel(Eigen::Vector3d _pos);
        Eigen::Vector3d ProjectiveGridPixel2Camera(Eigen::Vector2i _pos);
        int ObjectAssociation(Eigen::Vector4i _uvs);
        inline std::vector<ParticleFilter> Pedestrians(){return pedestrians_;}
    private:
        ///Camera
        Camera::Ptr camera_;
        GridMap::Ptr map_;
        std::vector<ParticleFilter> pedestrians_;
    };
}
#endif //PEDESTRIAN_TRACKING_FUSION_H