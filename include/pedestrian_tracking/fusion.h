#ifndef PEDESTRIAN_TRACKING_FUSION_H
#define PEDESTRIAN_TRACKING_FUSION_H

#include "ros/ros.h"
#include "image_transport/subscriber.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "cv_bridge/cv_bridge.h"
#include "ros/callback_queue.h"
#include "camera/camera.h"
#include "geometry_msgs/PointStamped.h"
#include "common/gaussian.h"

namespace luyifan{

#define ROWS 600
#define COLS 250
#define OriginX 125
#define OriginY 100

    class Fusion{
    public:
        Fusion(Camera::Ptr _camera);
        ~Fusion();

        void Process(cv::Mat _map, Eigen::Vector4i _uvs);
        Eigen::Vector2i ProjectiveCamera2GridPixel(Eigen::Vector3d _pos);
        Eigen::Vector2i GetPixelOnEdge(Eigen::Vector2i _begin, Eigen::Vector2i _end);
        Eigen::Vector2i Average(std::vector<Eigen::Vector2i> _As);
        void ApplyMaskToMap(cv::Mat _src, cv::Mat _mask);
        void ObjectExtractor(cv::Mat _src, std::vector<geometry_msgs::PointStamped>& _objects);
    private:
        ///Camera
        cv::Point pos_cam_;
        Camera::Ptr camera_;

    };
}
#endif //PEDESTRIAN_TRACKING_FUSION_H