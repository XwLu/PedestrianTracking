#include "pedestrian_tracking/fusion.h"

using namespace luyifan;
using namespace std;
using namespace Eigen;
using namespace cv;

Fusion::Fusion(Camera::Ptr _camera):
        camera_(_camera){
    ///Camera
    pos_cam_.x = OriginX;
    pos_cam_.y = OriginY;
}

Fusion::~Fusion() {

}

Vector2i Fusion::ProjectiveCamera2GridPixel(Eigen::Vector3d _pos) {
    Point origin(pos_cam_.x, ROWS-pos_cam_.y);
    Point xy_map(origin.x + 5*_pos(0), origin.y - 5*_pos(2));
    Vector2i result(xy_map.x, xy_map.y);
    return result;
}

Vector2i Fusion::GetPixelOnEdge(Eigen::Vector2i _begin, Eigen::Vector2i _end) {
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
        double y0 = _begin(1) + (COLS-_begin(0))*(_end(1) - _begin(1))/(_end(0) - _begin(0));
        if(y0 > 0)
            return Vector2i(COLS, y0);
        double x0 = _begin(0) - _begin(1)*(_end(0) - _begin(0))/(_end(1) - _begin(1));
        return Vector2i(x0, 0);
    }
}

Vector2i Fusion::Average(vector<Eigen::Vector2i> _As) {
    double sum1 = 0, sum2 = 0;
    for(auto A : _As){
        sum1 += A(0);
        sum2 += A(1);
    }
    return Vector2i(sum1/_As.size(), sum2/_As.size());
}

void Fusion::ApplyMaskToMap(cv::Mat _src, cv::Mat _mask) {
    assert(_src.size() == _mask.size());

    for(int i=0; i<_src.rows; ++i)
        for(int j=0; j<_src.cols; ++j){
            if(_mask.at<uchar>(i, j) < 255)
                _src.at<uchar>(i, j) = 0;
        }
}

void Fusion::ObjectExtractor(cv::Mat _src, vector<geometry_msgs::PointStamped>& _objects) {
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
        int map_y = ROWS - (boundRect[i].y + cvRound(boundRect[i].height/2));
        ///车前30米外的行人不考虑
        if(map_y > 200)
            continue;
        circle(_src, Point(boundRect[i].x, boundRect[i].y), 4, Scalar(255), 1);
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
    cout<<"find "<<_objects.size()<<" objects."<<endl;
    imshow("objects", _src);
    waitKey(5);
}

void Fusion::Process(cv::Mat _map, Eigen::Vector4i _uvs) {
    //imshow("grid_map_origin", _map);
    //waitKey(5);
    ///计算mask
    Mat mask3C(_map.size(), CV_8UC1, cv::Scalar(0));
    //_map.copyTo(mask3C);
    double z = 10;
    vector<Vector2i> edges = {};
    Vector3d xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(_uvs(0), _uvs(1)), z);
    Vector2i xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    Vector2i xy1 = GetPixelOnEdge(Vector2i(OriginX, ROWS-OriginY), Vector2i(xy_map));
    edges.emplace_back(xy1);
    line(mask3C, Point(OriginX, ROWS-OriginY), Point(xy1(0), xy1(1)), Scalar(255), 2, CV_AA);

    xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(_uvs(2), _uvs(3)), z);
    //cout<<"1: x: "<<xyz_cam(0)<<" y: "<<xyz_cam(1)<<" z: "<<xyz_cam(2)<<endl;
    xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    //cout<<"2: x: "<<xy_map(0)<<" y: "<<xy_map(1)<<endl;
    Vector2i xy2 = GetPixelOnEdge(Vector2i(125, 500), Vector2i(xy_map));
    //cout<<"3: x: "<<xy2(0)<<" y: "<<xy2(1)<<endl;
    edges.emplace_back(xy2);
    line(mask3C, Point(OriginX, ROWS-OriginY), Point(xy2(0), xy2(1)), Scalar(255), 2, CV_AA);
    edges.emplace_back(Vector2i(OriginX, ROWS-OriginY));

    //line(mask3C, Point(xy1(0), xy1(1)), Point(xy2(0), xy2(1)), Scalar(255), 2, CV_AA);
    Mat mask(_map.size(), CV_8UC1, cv::Scalar(0));
    threshold(mask3C, mask, 10, 255.0, CV_THRESH_BINARY);
    Vector2i seed = Average(edges);
    //circle(mask, Point(seed(0), seed(1)), 6, Scalar(255), 1);
    floodFill(mask, Point(seed(0), seed(1)), Scalar(255));
    //imshow("mask", mask);
    //waitKey(5);

    ///覆盖mask到原栅格图
    Mat grid_map;
    _map.copyTo(grid_map);
    ApplyMaskToMap(grid_map, mask);
    //imshow("grid_map", grid_map);
    //waitKey(5);

    ///提取栅格图中的候选目标
    vector<geometry_msgs::PointStamped> objects = {};
    ObjectExtractor(grid_map, objects);

    ///根据行人的像素高度估计深度值
    double depth = 7;

    ///计算候选点的置信度
    xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(0.5*(_uvs(0)+_uvs(2)), 0.5*(_uvs(1) + _uvs(3))), depth);
    xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    Gaussian gaussian(xy_map(0), ROWS-xy_map(1), 3, 30);
    for(auto object: objects){
        object.point.z = gaussian.GetValue(object.point.x, object.point.y);
        //cout<<"x: "<<object.point.x<<" y: "<<object.point.y<<" z: "<<object.point.z<<endl;
    }
    
}



