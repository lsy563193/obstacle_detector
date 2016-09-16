/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#define ARMA_DONT_USE_CXX11

#include <armadillo>
#include <obstacle_detector/Obstacles.h>

#include "../kalman.h"
#include "point.h"

namespace obstacle_detector
{

class TrackedObstacle {
public:
  TrackedObstacle(const CircleObstacle& init_obstacle, int fade_counter_size) : kf_(0, 3, 6) {
    obstacle_ = init_obstacle;
    obstacle_.tracked = true;

    if (obstacle_.obstacle_id == "" || obstacle_.obstacle_id == "-")
      obstacle_.obstacle_id = "O" + std::to_string(++obstacle_number_);

    fade_counter_size_ = fade_counter_size;

    // Initialize Kalman Filter structures
    kf_.A(0, 1) = TP_;
    kf_.A(2, 3) = TP_;
    kf_.A(4, 5) = TP_;

    kf_.C(0, 0) = 1.0;
    kf_.C(1, 2) = 1.0;
    kf_.C(2, 4) = 1.0;

    kf_.q_pred(0) = obstacle_.center.x;
    kf_.q_pred(1) = obstacle_.velocity.x;
    kf_.q_pred(2) = obstacle_.center.y;
    kf_.q_pred(3) = obstacle_.velocity.y;
    kf_.q_pred(4) = obstacle_.radius;

    kf_.q_est(0) = obstacle_.center.x;
    kf_.q_est(1) = obstacle_.velocity.x;
    kf_.q_est(2) = obstacle_.center.y;
    kf_.q_est(3) = obstacle_.velocity.y;
    kf_.q_est(4) = obstacle_.radius;
  }

  ~TrackedObstacle() {
//    obstacle_number_--;
  }

  void setCovariances(double process_var, double measurement_var) {
    kf_.R(0, 0) = measurement_var;
    kf_.R(1, 1) = measurement_var;
    kf_.R(2, 2) = measurement_var;

    kf_.Q(0, 0) = process_var;
    kf_.Q(1, 1) = process_var * 10.0;
    kf_.Q(2, 2) = process_var;
    kf_.Q(3, 3) = process_var * 10.0;
    kf_.Q(4, 4) = process_var;
    kf_.Q(5, 5) = process_var * 10.0;
  }

  void updateMeasurement(const CircleObstacle& new_obstacle) {
    kf_.y(0) = new_obstacle.center.x;
    kf_.y(1) = new_obstacle.center.y;
    kf_.y(2) = new_obstacle.radius;

    fade_counter_ = fade_counter_size_;
  }

  void updateTracking() {
    kf_.updateState();

    obstacle_.center.x = kf_.q_est(0);
    obstacle_.center.y = kf_.q_est(2);

    obstacle_.velocity.x = kf_.q_est(1);
    obstacle_.velocity.y = kf_.q_est(3);

    obstacle_.radius = kf_.q_est(4);

    fade_counter_--;
  }

  void setFused() { fused_ = true; }
  void setFissed() { fissed_ = true; }

  CircleObstacle getObstacle() const { return obstacle_; }
  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }

private:
  KalmanFilter kf_;
  CircleObstacle obstacle_;

  static int obstacle_number_;
  static const double TP_;      // Sampling time in sec.

  int fade_counter_size_;
  int fade_counter_;            // If the fade counter reaches 0, remove the obstacle from the list

  bool fused_;
  bool fissed_;
};

}
