#include "pedestrian_tracking/fusion.h"

using namespace luyifan;
using namespace std;
using namespace Eigen;
using namespace cv;

Fusion::Fusion(Camera::Ptr _camera, GridMap::Ptr _map):
        camera_(_camera), map_(_map){

}

Fusion::~Fusion() {}

Vector2i Fusion::ProjectiveCamera2GridPixel(Eigen::Vector3d _pos) {
    Point origin(OriginX, map_->Rows()-OriginY);
    Point xy_map(origin.x + _pos(0)*map_->InvResolving(), origin.y - _pos(2)*map_->InvResolving());
    Vector2i result(xy_map.x, xy_map.y);
    return result;
}

Vector3d Fusion::ProjectiveGridPixel2Camera(Eigen::Vector2i _pos) {
    Point origin(OriginX, map_->Rows()-OriginY);
    return Vector3d(
            (_pos(0) - origin.x)*map_->Resolving(),
            0,
            (origin.y - _pos(1))*map_->Resolving()
    );
}

int Fusion::ObjectAssociation(Eigen::Vector4i _uvs) {
    int id = 0;
    for(auto pedestrian : pedestrians_){
        Vector2i pix = map_->Map2Pixel(Eigen::Vector2i(pedestrian.Result().position.x, pedestrian.Result().position.y));
        Vector3d cam = ProjectiveGridPixel2Camera(pix);
        Vector2i uv = camera_->ProjectiveCamera2Pixel(cam);
        //距离小于10个像素就是同一个人
        if(fabs(uv(0) - 0.5*(_uvs(0) + _uvs(2))) < 10)
            return id;
        id++;
    }
    return -1;//新出现的人
}

void Fusion::Show(const cv::Mat& _map) {
    Mat grid_map;
    _map.copyTo(grid_map);
    //dilate(grid_map, grid_map, getStructuringElement(MORPH_ELLIPSE, Size(4, 4)));
    //cout<<"draw particles"<<endl;
    for(auto ped:pedestrians_){
        vector<geometry_msgs::Pose> particles = ped.Particles();
        for(auto p:particles){
            auto p_map_pix = map_->Map2Pixel(Vector2i(p.position.x, p.position.y));
            //cout<<"x: "<<p_map_pix(0)<<" y: "<<p_map_pix(1)<<endl;
            circle(grid_map, Point(p_map_pix(0), p_map_pix(1)), 1, Scalar(255), 1);
        }
    }
    imshow("map with particles", grid_map);
    waitKey(5);
    //cout<<"draw pedestrians"<<endl;
    for(auto ped:pedestrians_){
        auto map_pix = map_->Map2Pixel(Eigen::Vector2i(ped.TmpResult().position.x, ped.TmpResult().position.y));
        circle(grid_map, Point(map_pix(0), map_pix(1)), 4, Scalar(255), 1);
    }
    imshow("map with pedestrians", grid_map);
    waitKey(5);
}


void Fusion::Process(const cv::Mat& _map, const Eigen::Vector4i& _uvs) {
    //imshow("grid_map_origin", _map);
    //waitKey(5);
    ///计算mask
    Mat mask3C(_map.size(), CV_8UC1, cv::Scalar(0));
    double z = 10;
    vector<Vector2i> edges = {};
    Vector3d xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(_uvs(0), _uvs(1)), z);
    Vector2i xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    Vector2i xy1 = map_->GetPixelOnEdge(Vector2i(OriginX, map_->Rows()-OriginY), Vector2i(xy_map));
    edges.emplace_back(xy1);
    line(mask3C, Point(OriginX, map_->Rows()-OriginY), Point(xy1(0), xy1(1)), Scalar(255), 2, CV_AA);

    xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(_uvs(2), _uvs(3)), z);
    xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    Vector2i xy2 = map_->GetPixelOnEdge(Vector2i(125, 500), Vector2i(xy_map));
    edges.emplace_back(xy2);
    line(mask3C, Point(OriginX, map_->Rows()-OriginY), Point(xy2(0), xy2(1)), Scalar(255), 2, CV_AA);
    edges.emplace_back(Vector2i(OriginX, map_->Rows()-OriginY));

    line(mask3C, Point(xy1(0), xy1(1)), Point(xy2(0), xy2(1)), Scalar(255), 2, CV_AA);
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
    map_->ApplyMaskToMap(grid_map, mask);
    //imshow("grid_map", grid_map);
    //waitKey(5);

    ///提取栅格图中的候选目标
    vector<geometry_msgs::PointStamped> candidates = {};
    map_->ObjectExtractor(grid_map, candidates);

    ///根据行人的像素高度估计深度值
    double depth = 7;

    ///计算候选点的置信度
    xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(0.5*(_uvs(0)+_uvs(2)), 0.5*(_uvs(1) + _uvs(3))), depth);
    xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    Gaussian gconf(0, 0, 8, 80);
    for(auto& object: candidates){
        double lat = Dist2Line(Vector2d(OriginX, OriginY), Vector2d(xy_map(0), map_->Rows()-xy_map(1)), Vector2d(object.point.x, object.point.y));
        double lon = pow(lat*lat + pow(object.point.x-xy_map(0), 2.0) + pow(object.point.y-map_->Rows()+xy_map(1), 2.0), 0.5);
        object.point.z = gconf.GetValue(lat, lon);
        //cout<<"x: "<<object.point.x<<" y: "<<object.point.y<<" z: "<<object.point.z<<endl;
        cout<<"x: "<<lat<<" y: "<<lon<<" z: "<<object.point.z<<endl;
    }

    if(candidates.empty()){
        ROS_ERROR("no candidates in laser scanner!!");
        return;
    }

    ///确定是否是新的目标
    int index = ObjectAssociation(_uvs);
    //新出现的人
    if(index < 0){
        cout<<"New pedestrian show up!"<<candidates.size()<<" candidates detected!"<<endl;
        ParticleFilter pedestrian(Partcile_NUM, candidates);//初始化粒子滤波
        pedestrians_.emplace_back(pedestrian);
    } else{
        pedestrians_[index].Update(candidates);
    }

}



