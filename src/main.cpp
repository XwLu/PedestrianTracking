#include "pedestrian_tracking/fusion.h"
#include "common/my_mutex.h"

using namespace std;
#define IMAGE_ROWS 720.0
#define IMAGE_COLS 1280.0

cv::Mat global_grid_map;
boost::shared_mutex mutex_map;
Eigen::Vector4i global_uvs;

luyifan::Fusion* fusion;

void GridMapCallback(const sensor_msgs::ImageConstPtr &_map) {
    //cout<<"image!"<<endl;
    luyifan::write_lock wlock(mutex_map);
    cv_bridge::toCvShare(_map,"mono8")->image.copyTo(global_grid_map);

    for(int row = 0; row < global_grid_map.rows; row++){
        for(int col = 0; col < global_grid_map.cols; col++){
            if(global_grid_map.at<uchar>(row, col) > 0){
                global_grid_map.at<uchar>(row, col) = 255;
            }
        }
    }
    fusion->Show(global_grid_map);
}

void BoundingBoxCallback(const std_msgs::Float64MultiArrayConstPtr& _bbxs) {
    //cout<<"bbx: "<<_bbxs->data.size()<<endl;

    if(global_grid_map.empty())
        return;
    luyifan::read_lock rlock(mutex_map);

    ///获取bbx
    double u_left_top, v_left_top, u_right_bottom, v_right_bottom;
    u_left_top = IMAGE_ROWS/2 - 60;//TODO: 调试用
    v_left_top = IMAGE_COLS/2;
    u_right_bottom = IMAGE_ROWS/2 + 60;
    v_right_bottom = IMAGE_COLS/2;

    uint64_t num = _bbxs->data.size()/4;
    if(!num)
        return;
    ///处理
    for(int i=0; i<num; ++i){
        cout<<"processing "<<i+1<<" pedestrian."<<endl;
        u_left_top = _bbxs->data[i*4];
        v_left_top = _bbxs->data[i*4+1];
        u_right_bottom = _bbxs->data[i*4+2];
        v_right_bottom = _bbxs->data[i*4+3];

        global_uvs<<u_left_top, v_left_top, u_right_bottom, v_right_bottom;
        fusion->Process(global_grid_map, global_uvs);
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "fusion");
    ros::NodeHandle nh;
    ros::CallbackQueue queue_1;
    ros::CallbackQueue queue_2;

    nh.setCallbackQueue(&queue_1);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_grid_map_;
    sub_grid_map_ = it.subscribe("/grid_map_origin", 1, GridMapCallback);
    nh.setCallbackQueue(&queue_2);
    ros::Subscriber sub_bbx_ = nh.subscribe("/pedestrian", 1, BoundingBoxCallback);

    luyifan::Camera::Ptr camera(new luyifan::Camera(IMAGE_COLS/2, IMAGE_ROWS/2, 830.0, 830.0));
    luyifan::GridMap::Ptr grid_map(new luyifan::GridMap(600, 250, 125, 100, 0.2));
    fusion = new luyifan::Fusion(camera, grid_map);

    cout<<"MultiThread Receiving!"<<endl;
    ros::AsyncSpinner spinner_1(1, &queue_1);
    spinner_1.start();
    ros::AsyncSpinner spinner_2(1, &queue_2);
    spinner_2.start();

    ros::waitForShutdown();
}