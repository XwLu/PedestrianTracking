//
// Created by luyifan on 18-12-18.
//

#include "pedestrian_tracking/ParticleFilter.h"

using namespace luyifan;
using namespace std;
using namespace Eigen;
using namespace cv;

ParticleFilter::ParticleFilter(int _num, vector<geometry_msgs::PointStamped>& _candidates):
num_(_num), mean_(0.0), stddev_(2.0){
    sort(_candidates.begin(), _candidates.end(), [](const geometry_msgs::PointStamped& p1,
                                                    const geometry_msgs::PointStamped& p2){
        return p1.point.z > p2.point.z;
    });
    result_.position.x = _candidates.front().point.x;
    result_.position.y = _candidates.front().point.y;
    tmp_result_ = result_;
    vel_.linear.x = vel_.linear.y = 0;
    particles_.resize(_num);
    ParticlesInit(_candidates);
    noise_ = std::normal_distribution<double>(mean_, stddev_);
    generator_ = std::default_random_engine(time(0));//避免每次循环都一样
}

ParticleFilter::~ParticleFilter() {}

vector<pair<int, Vector2d>> ParticleFilter::DistributeParticles(const vector<geometry_msgs::PointStamped>& _candidates) {
    ///默认进来的_candidates已经降序排列了
    vector<pair<int, Vector2d>> res = {};
    double confidences = 0;
    for(auto can : _candidates){
        confidences += can.point.z;
    }

    int sum = 0;
    for(auto can : _candidates){
        int n =cvRound(num_ * can.point.z / confidences);
        //cout<<"num particle: "<<n<<endl;
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

void ParticleFilter::ScatterParticles(const vector<pair<int, Vector2d>>& _distribution) {
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

void ParticleFilter::ParticlesInit(const std::vector<geometry_msgs::PointStamped>& _candidates) {
    vector<pair<int, Vector2d>> distribution = DistributeParticles(_candidates);
    ScatterParticles(distribution);
}

double ParticleFilter::EvaluateParticle(std::vector<geometry_msgs::PointStamped> _candidates,
                                        geometry_msgs::Pose _particle) {
    double scores = 0;
    for(auto can: _candidates){
        //距离最小是0.1
        double distance = max(0.1, sqrt(pow(can.point.x - _particle.position.x, 2.0) +
                                        pow(can.point.y - _particle.position.y, 2.0)));
        double score = can.point.z / distance;
        scores += score;
    }
    return scores;
}

void ParticleFilter::NormalizeConfidence(std::vector<geometry_msgs::Pose> &_particles,
                                         double& sum_con) {
    double k = 1/sum_con;
    sum_con = 0;
    for(auto& p : _particles){
        p.position.z = k * p.position.z;
        sum_con += p.position.z;
    }
}

void ParticleFilter::Resampling(std::vector<geometry_msgs::Pose> &_particles, const double _sum_conf) {
    vector<pair<int, double>> id_conf_pairs;
    double conf = 0;
    for(int i=0; i<_particles.size(); ++i){
        conf += _particles[i].position.z;
        id_conf_pairs.emplace_back(pair<int, double>(i, conf));
        //cout<<"conf: "<<_particles[i].position.z<<endl;
    }

    //选择新的粒子
    srand((unsigned int)time(0));
    vector<geometry_msgs::Pose> particles = {};
    for(int i=0; i<_particles.size(); ++i){
        double seed = rand()%100/100.0;
        for(auto pair : id_conf_pairs){
            if(seed < pair.second){
                particles.emplace_back(_particles[pair.first]);
                particles.back().position.z = 1.0 / _particles.size();
                break;
            }
        }
    }
    //cout<<"new: "<<particles.size()<<endl;
    _particles = particles;
}

geometry_msgs::Pose ParticleFilter::WeightedAverage(const std::vector<geometry_msgs::Pose> &_particles) {
    geometry_msgs::Pose tmp;
    for(auto p:_particles){
        tmp.position.x += p.position.z * p.position.x;
        tmp.position.y += p.position.z * p.position.y;
    }
    return tmp;
}

void ParticleFilter::Update(const std::vector<geometry_msgs::PointStamped>& _candidates) {
    if(_candidates.empty()){
        cout<<"No measurement!"<<endl;
        return;
    }

    /**********SIR PF***********/
    srand((unsigned int)time(0));
    double confs = 0;
    for(auto& p : particles_){
        //prediction
        double delta_x = noise_(generator_);
        double delta_y = noise_(generator_);
        p.position.x += vel_.linear.x + delta_x;
        p.position.y += vel_.linear.y + delta_y;
        //cout<<"noise x: "<<delta_x<<" y: "<<delta_y<<endl;
        //updating
        p.position.z = EvaluateParticle(_candidates, p);
        confs += p.position.z;
    }
    //cout<<"confs: "<<confs<<endl;
    NormalizeConfidence(particles_, confs);
    Resampling(particles_, confs);
    geometry_msgs::Pose res = WeightedAverage(particles_);
    ///更新速度,行人的速度不大于4m/s,每个循环0.1s(2个像素)
    vel_.linear.x = VelocityLimit(res.position.x - tmp_result_.position.x, 1);
    vel_.linear.y = VelocityLimit(res.position.y - tmp_result_.position.y, 1);
    //cout<<"v_x: "<<vel_.linear.x<<" v_y: "<<vel_.linear.y<<endl;
    tmp_result_ = res;
}