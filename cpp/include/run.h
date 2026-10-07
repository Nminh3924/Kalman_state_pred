#pragma once

#include <functional>
#include <vector>

#include <Eigen/Dense>

#include "filters/filter.h"
#include "io.h"

struct Estimate {
    double t;
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    double nis;  // of the GPS fix used at this time, NaN if there was none
};

// Drives a filter through time-sorted measurements. It is initialized from the
// first pair of GPS fixes the filter accepts, with the IMU bridging the time
// between them; after that every IMU sample sets the input used to predict,
// and every GPS fix updates.
// Fixes whose NIS exceeds gate are ignored as outliers. With a gate, a filter
// reports only once it has accepted a fix, and it is initialized again if its
// first fix, or several fixes in a row, are rejected (see run.cpp).
// Returns one estimate per distinct timestamp at which the filter reports.
//
// observer, if given, is called each time an estimate is reported, with the
// time, the filter in that state and the latest IMU sample.
using Observer = std::function<void(double t, const Filter& filter, const Eigen::Vector2d& u)>;

std::vector<Estimate> run(Filter& filter, const std::vector<Measurement>& measurements,
                          double gate = NO_GATE, const Observer& observer = nullptr);
