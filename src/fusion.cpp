#include "pedestrian_tracking/fusion.h"

using namespace luyifan;
using namespace std;
using namespace Eigen;
using namespace cv;

Fusion::Fusion(Camera::Ptr _camera, GridMap::Ptr _map):
        camera_(_camera), map_(_map), life_time_(3.0){}

Fusion::~Fusion() {}

Vector2i Fusion::ProjectiveCamera2GridPixel(const Eigen::Vector3d& _pos) {
    Point origin(CameraX, map_->Rows()-CameraY);
    Point xy_map(origin.x + _pos(0)*map_->InvResolving(), origin.y - _pos(2)*map_->InvResolving());
    Vector2i result(xy_map.x, xy_map.y);
    return result;
}

Vector3d Fusion::ProjectiveGridPixel2Camera(const Eigen::Vector2i& _pos) {
    Point origin(CameraX, map_->Rows()-CameraY);
    return Vector3d(
            (_pos(0) - origin.x)*map_->Resolving(),
            0,
            (origin.y - _pos(1))*map_->Resolving()
    );
}

void Fusion::DeleteOldPedestrians() {
    for(auto iter = pedestrians_.begin(); iter != pedestrians_.end();){
        if(iter->TimeNotInView(ros::Time::now()) > life_time_)
            iter = pedestrians_.erase(iter);
        else
            iter++;
    }
}

vector<pair<int, Eigen::Vector4i>> Fusion::ObjectAssociation(const vector<Eigen::Vector4i>& _uvs) {
    assert(!_uvs.empty());
    ///暴力匹配
    vector<pair<double, pair<int, int>>> maps = {};
    ///每种组合求距离
    for(int i=0; i<_uvs.size(); i++){
        for(int j=0; j<pedestrians_.size(); j++){
            Vector3d xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(0.5*(_uvs[i](0)+_uvs[i](2)), 0.5*(_uvs[i](1) + _uvs[i](3))), 50);
            Vector2i xy_map = ProjectiveCamera2GridPixel(xyz_cam);
            double distance = Dist2Line(Vector2d(CameraX, CameraY), Vector2d(xy_map(0), map_->Rows()-xy_map(1)), Vector2d(pedestrians_[j].Result().position.x, pedestrians_[j].Result().position.y));
            //cout<<"uv: "<<i<<" ped: "<<j<<" distance: "<<distance<<endl;
            maps.emplace_back(make_pair(distance, pair<int, int>(i, j)));
        }
    }
    sort(maps.begin(), maps.end(),[](const pair<double, pair<int, int>> a, pair<double, pair<int, int>>b){
        return a.first<b.first;
    });
    ///提取组合
    vector<pair<int, int>> pairs = {};
    for(int i=0; i<min(_uvs.size(), pedestrians_.size()); i++){
        if(maps.empty())
            break;
        if(maps.begin()->first > 20.0){
            ROS_ERROR("Distance too large. Break directly!");
            break;
        }
        pair<int, int> p = maps.begin()->second;
        int first = p.first;
        int second = p.second;
        pairs.emplace_back(p);
        maps.erase(maps.begin());
        for(auto iter=maps.begin(); iter!=maps.end();){
            if(iter->second.first == first || iter->second.second == second)
                iter = maps.erase(iter);
            else
                iter++;
        }
    }
    ///存入已经提取的组合
    vector<pair<int, Eigen::Vector4i>> associations = {};
    for(auto& p:pairs){
        associations.emplace_back(pair<int, Eigen::Vector4i>(p.second, _uvs[p.first]));
    }
    ///新目标的组合
    bool bNew;
    for(int i=0; i<_uvs.size(); i++){
        bNew = true;
        for(auto& p:pairs){
            if(i==p.first){
                bNew = false;
                break;
            }
        }
        if(bNew){
            associations.emplace_back(pair<int, Eigen::Vector4i>(-1, _uvs[i]));
        }
    }
    return associations;
}

void Fusion::Show(const cv::Mat& _map) {
    Mat grid_map;
    _map.copyTo(grid_map);
    //dilate(grid_map, grid_map, getStructuringElement(MORPH_ELLIPSE, Size(4, 4)));
    /*
    //cout<<"Draw particles start"<<endl;
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
    //cout<<"Draw particles end!"<<endl;
    */

    //cout<<"draw pedestrians"<<endl;
    for(auto ped:pedestrians_){
        auto map_pix = map_->Map2Pixel(Eigen::Vector2i(ped.TmpResult().position.x, ped.TmpResult().position.y));
        circle(grid_map, Point(map_pix(0), map_pix(1)), 4, Scalar(255), 1);
    }
    imshow("map with pedestrians", grid_map);
    waitKey(5);

}


void Fusion::Process(const cv::Mat& _map, const pair<int, Eigen::Vector4i>& _pair) {
    boost::timer timer_detection;
    Eigen::Vector4i _uvs = _pair.second;
    //imshow("grid_map_origin", _map);
    //waitKey(5);
    ///计算mask
    Mat mask3C(_map.size(), CV_8UC1, cv::Scalar(0));
    double z = (map_->Rows() - CameraY) * map_->InvResolving() * 0.75;
    vector<Vector2i> edges = {};
    Vector3d xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(_uvs(0), _uvs(1)), z);

    Vector2i xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    Vector2i xy1 = map_->GetPixelOnEdge(Vector2i(CameraX, map_->Rows()-CameraY), Vector2i(xy_map));
    edges.emplace_back(xy1);
    line(mask3C, Point(CameraX, map_->Rows()-CameraY), Point(xy1(0), xy1(1)), Scalar(255), 2, CV_AA);

    xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(_uvs(2), _uvs(3)), z);
    xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    Vector2i xy2 = map_->GetPixelOnEdge(Vector2i(CameraX, map_->Rows() - CameraY), Vector2i(xy_map));
    edges.emplace_back(xy2);
    line(mask3C, Point(CameraX, map_->Rows()-CameraY), Point(xy2(0), xy2(1)), Scalar(255), 2, CV_AA);
    edges.emplace_back(Vector2i(CameraX, map_->Rows()-CameraY));

    line(mask3C, Point(xy1(0), xy1(1)), Point(xy2(0), xy2(1)), Scalar(255), 2, CV_AA);
    Mat mask(_map.size(), CV_8UC1, cv::Scalar(0));
    threshold(mask3C, mask, 10, 255.0, CV_THRESH_BINARY);
    Vector2i seed = Average(edges);
    //circle(mask, Point(seed(0), seed(1)), 6, Scalar(255), 1);
    //cout<<"seed x: "<<seed(0)<<" y: "<<seed(1)<<endl;
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
    cout<<"Lidar detection costs time: "<<timer_detection.elapsed()<<" s."<<endl;
    boost::timer timer_tracking;
    ///根据行人的像素高度估计深度值
    double fy = camera_->K()(1,1);
    double h = _uvs(3)-_uvs(1) + 50;//50是调试出来的修正值
    double depth = fy * 1.7 / h;
    ///镜头内的人距离摄像头太近，无法看到整个人，该方法失效(可以用宽度近似估计)
    if(h > 410){
        depth = 4.0;
    }
    //cout<<"h: "<<(_uvs(3)-_uvs(1))<<" depth: "<<depth<<endl;

    ///计算候选点的置信度
    xyz_cam = camera_->ProjectivePixel2Camera(Vector2d(0.5*(_uvs(0)+_uvs(2)), 0.5*(_uvs(1) + _uvs(3))), depth);
    xy_map = ProjectiveCamera2GridPixel(xyz_cam);
    Gaussian gconf(0, 0, 8, 80);
    for(auto& object: candidates){
        double lat = Dist2Line(Vector2d(CameraX, CameraY), Vector2d(xy_map(0), map_->Rows()-xy_map(1)), Vector2d(object.point.x, object.point.y));
        double lon = pow(lat*lat + pow(object.point.x-xy_map(0), 2.0) + pow(object.point.y-map_->Rows()+xy_map(1), 2.0), 0.5);
        object.point.z = gconf.GetValue(lat, lon);
        //cout<<"x: "<<lat<<" y: "<<lon<<" z: "<<object.point.z<<endl;
    }

    /*
    Mat tmp = _map.clone();
    circle(tmp, Point(xy_map(0), xy_map(1)), 2, Scalar(255), 1);
    imshow("estimate", tmp);
    waitKey(5);
    */

    if(candidates.empty()){
        ROS_ERROR("no candidates in laser scanner!!");
        return;
    }

    //新出现的人
    if(_pair.first < 0){
        cout<<"New pedestrian show up!"<<candidates.size()<<" candidates detected!"<<endl;
        ParticleFilter pedestrian(Partcile_NUM, candidates);//初始化粒子滤波
        pedestrians_.emplace_back(pedestrian);
    } else{
        pedestrians_[_pair.first].Update(candidates);
    }
    cout<<"Tracking costs time: "<<timer_tracking.elapsed()<<" s."<<endl;
}



