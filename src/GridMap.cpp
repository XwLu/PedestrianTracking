//
// Created by luyifan on 18-12-18.
//

#include "grid_map/GridMap.h"

using namespace luyifan;
using namespace Eigen;
using namespace cv;
using namespace std;

GridMap::GridMap(int _rows, int _cols, int _origin_x, int _origin_y, double _resolving):
rows_(_rows), cols_(_cols), origin_x_(_origin_x), origin_y_(_origin_y), resolving_(_resolving){
    inv_resolving_ = 1/resolving_;
}

GridMap::~GridMap() {}

Eigen::Vector2i GridMap::Pixel2Map(Eigen::Vector2i _pos) {
    return Eigen::Vector2i(
      _pos(0),
      rows_ - _pos(1)
    );
}

Eigen::Vector2i GridMap::Map2Pixel(Eigen::Vector2i _pos) {
    return Eigen::Vector2i(
      _pos(0),
      rows_ - _pos(1)
    );
}

Vector2i GridMap::GetPixelOnEdge(Eigen::Vector2i _begin, Eigen::Vector2i _end) {
    if(_end(0) == _begin(0))
        return Vector2i(_begin(0), 0);
    else if(_end(0) < _begin(0)){
        double y0 = _begin(1) - _begin(0)*(_end(1) - _begin(1))/(_end(0) - _begin(0));
        if(y0 > 0)
            return Vector2i(0, y0);
        double x0 = _begin(0) - _begin(1)*(_end(0) - _begin(0))/(_end(1) - _begin(1));
        return Vector2i(x0, 0);
    }
    else{
        double y0 = _begin(1) + (cols_-_begin(0))*(_end(1) - _begin(1))/(_end(0) - _begin(0));
        if(y0 > 0)
            return Vector2i(cols_, y0);
        double x0 = _begin(0) - _begin(1)*(_end(0) - _begin(0))/(_end(1) - _begin(1));
        return Vector2i(x0, 0);
    }
}

void GridMap::ObjectExtractor(const cv::Mat& _src, vector<geometry_msgs::PointStamped>& _objects) {
    Mat tmp;
    dilate(_src, tmp, getStructuringElement(MORPH_ELLIPSE, Size(4, 4)));
    //imshow("grid_map_dilate", tmp);
    //waitKey(5);
    ///find external contours in grid map
    vector<vector<Point>> contours = {};
    findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    ///bbx of contours
    vector<Rect> boundRect(contours.size());
    unsigned int id = 0;
    for(int i=0; i<contours.size(); i++){
        boundRect[i] = boundingRect(Mat(contours[i]));
        if(boundRect[i].area() <= 0 || boundRect[i].area() > 100 ||
           boundRect[i].height > 10 || boundRect[i].width > 10)
            continue;

        int map_x = boundRect[i].x + cvRound(boundRect[i].width/2);
        int map_y = rows_ - (boundRect[i].y + cvRound(boundRect[i].height/2));
        ///车前30米外的行人不考虑
        if(map_y > 200)
            continue;
        ///画出检测到的候选点
        circle(tmp, Point(boundRect[i].x, boundRect[i].y), 4, Scalar(255), 1);
        //cout<<"x: "<<map_x<<" y: "<<map_y<<endl;
        geometry_msgs::PointStamped object;
        object.header.frame_id = "grid_map";
        object.header.stamp = ros::Time::now();
        object.header.seq = id++;
        object.point.x = map_x;
        object.point.y = map_y;
        object.point.z = 0;///该点是行人的置信度
        _objects.emplace_back(object);
    }
    //circle(tmp, Point(126, 492), 2, Scalar(255), 1);
    //cout<<"find "<<_objects.size()<<" objects."<<endl;
    //imshow("objects", tmp);
    //waitKey(5);
}


void GridMap::ApplyMaskToMap(cv::Mat _src, cv::Mat _mask) {
    assert(_src.size() == _mask.size());
    for(int i=0; i<_src.rows; ++i)
        for(int j=0; j<_src.cols; ++j){
            if(_mask.at<uchar>(i, j) < 255)
                _src.at<uchar>(i, j) = 0;
        }
}