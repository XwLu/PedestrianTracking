//
// Created by luyifan on 18-12-17.
//
#include "common/gaussian.h"

using namespace luyifan;

Gaussian::Gaussian(double _e1, double _e2, double _v1, double _v2):
        expect1_(_e1), expect2_(_e2), variance1_(_v1), variance2_(_v2) {
    variance1_2_ = variance1_ * variance1_;
    variance2_2_ = variance2_ * variance2_;
}

Gaussian::~Gaussian(){}

