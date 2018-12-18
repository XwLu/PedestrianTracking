//
// Created by luyifan on 18-12-18.
//

#include "pedestrian_tracking/ParticleFilter.h"

using namespace luyifan;
using namespace std;
using namespace Eigen;

ParticleFilter::ParticleFilter(int _num, vector<geometry_msgs::PointStamped> _candidates):
num_(_num){
    sort(_candidates.begin(), _candidates.end(), [](const geometry_msgs::PointStamped& p1,
                                                    const geometry_msgs::PointStamped& p2){
        return p1.point.z > p2.point.z;
    });
    result_.position.x = _candidates.front().point.x;
    result_.position.y = _candidates.front().point.y;
    vel_.linear.x = vel_.linear.y = 0;
    particles_.resize(_num);
    ParticlesInit(_candidates);
}

ParticleFilter::~ParticleFilter() {}

vector<pair<int, Vector2d>> ParticleFilter::DistributeParticles(vector<geometry_msgs::PointStamped> _candidates) {
    ///默认进来的_candidates已经降序排列了
    vector<pair<int, Vector2d>> res = {};
    double confidences = 0;
    for(auto can : _candidates){
        confidences += can.point.z;
    }

    int sum = 0;
    for(auto can : _candidates){
        int n =cvRound(num_ * can.point.z / confidences);
        cout<<"num particle: "<<n<<endl;
        sum += n;
        if(sum > num_ || n == 0)
            break;
        if(n > 0)
            res.emplace_back(pair<int, Vector2d> (n, Vector2d(can.point.x, can.point.y)));
    }
    if(sum < num_)
        res.back().first + (num_ - sum);
    return res;
}

void ParticleFilter::ScatterParticles(vector<pair<int, Vector2d>> _distribution) {
    srand((unsigned int)time(0));
    int sum = 0;
    for(auto dis : _distribution){
        int i = sum;
        for(; i<sum + dis.first; ++i){
            particles_[i].position.x = dis.second(0) + rand()%16 - 8;
            particles_[i].position.y = dis.second(1) + rand()%16 - 8;
        }
        sum = i;
    }
}

void ParticleFilter::ParticlesInit(std::vector<geometry_msgs::PointStamped> _candidates) {
    vector<pair<int, Vector2d>> distribution = DistributeParticles(_candidates);
    ScatterParticles(distribution);
}

void ParticleFilter::Update(std::vector<geometry_msgs::PointStamped> _candidates) {

}