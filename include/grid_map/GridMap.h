//
// Created by luyifan on 18-12-18.
//

#ifndef PEDESTRIAN_TRACKING_GRIDMAP_H
#define PEDESTRIAN_TRACKING_GRIDMAP_H

#include <memory>
#include <Eigen/Core>
#include "opencv2/opencv.hpp"
#include "geometry_msgs/PointStamped.h"

namespace luyifan{
    class GridMap{
    public:
        typedef std::shared_ptr<GridMap> Ptr;
        GridMap(int _rows, int _cols, int _origin_x, int _origin_y, double _resolving);
        ~GridMap();

        inline int Rows() { return rows_; }
        inline int Cols() { return cols_; }
        inline int OriginX() { return origin_x_; }
        inline int OriginY() { return origin_y_; }
        inline double Resolving() { return resolving_; }
        inline double InvResolving() { return inv_resolving_;}

        Eigen::Vector2i GetPixelOnEdge(const Eigen::Vector2i& _begin, const Eigen::Vector2i& _end);
        void ObjectExtractor(const cv::Mat& _src, std::vector<geometry_msgs::PointStamped>& _objects);
        void ApplyMaskToMap(cv::Mat _src, cv::Mat _mask);
        Eigen::Vector2i Pixel2Map(const Eigen::Vector2i& _pos);
        Eigen::Vector2i Map2Pixel(const Eigen::Vector2i& _pos);

    private:
        int rows_;
        int cols_;
        int origin_x_;
        int origin_y_;
        double resolving_;
        double inv_resolving_;
    };
}

#endif //PEDESTRIAN_TRACKING_GRIDMAP_H
