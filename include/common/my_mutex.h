//
// Created by luyifan on 18-12-16.
//

#ifndef PEDESTRIAN_TRACKING_MY_MUTEX_H
#define PEDESTRIAN_TRACKING_MY_MUTEX_H

#include <mutex>

namespace luyifan{
    typedef boost::shared_lock<boost::shared_mutex> read_lock;
    typedef boost::unique_lock<boost::shared_mutex> write_lock;
}

#endif //PEDESTRIAN_TRACKING_MY_MUTEX_H
