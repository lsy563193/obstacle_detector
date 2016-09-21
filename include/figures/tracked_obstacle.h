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
  TrackedObstacle(const CircleObstacle& init_obstacle) : kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_r_(0, 1, 2) {
    obstacle_ = init_obstacle;
    obstacle_.tracked = true;
    fade_counter_ = fade_counter_size_;

    if (obstacle_.obstacle_id == "" || obstacle_.obstacle_id == "-")
      obstacle_.obstacle_id = "O" + std::to_string(++obstacle_number_);

    // Initialize Kalman filter structures
    kf_x_.A(0, 1) = sampling_time_;
    kf_x_.C(0, 0) = 1.0;
    kf_x_.R(0, 0) = measurement_variance_;
    kf_x_.Q(0, 0) = process_variance_;
    kf_x_.Q(1, 1) = process_rate_variance_;
    kf_x_.q_pred(0) = obstacle_.center.x;
    kf_x_.q_pred(1) = obstacle_.velocity.x;
    kf_x_.q_est(0) = obstacle_.center.x;
    kf_x_.q_est(1) = obstacle_.velocity.x;

    kf_y_.A(0, 1) = sampling_time_;
    kf_y_.C(0, 0) = 1.0;
    kf_y_.R(0, 0) = measurement_variance_;
    kf_y_.Q(0, 0) = process_variance_;
    kf_y_.Q(1, 1) = process_rate_variance_;
    kf_y_.q_pred(0) = obstacle_.center.y;
    kf_y_.q_pred(1) = obstacle_.velocity.y;
    kf_y_.q_est(0) = obstacle_.center.y;
    kf_y_.q_est(1) = obstacle_.velocity.y;

    kf_r_.A(0, 1) = sampling_time_;
    kf_r_.C(0, 0) = 1.0;
    kf_r_.R(0, 0) = measurement_variance_;
    kf_r_.Q(0, 0) = process_variance_;
    kf_r_.Q(1, 1) = process_rate_variance_;
    kf_r_.q_pred(0) = obstacle_.radius;
    kf_r_.q_est(0) = obstacle_.radius;
  }

  ~TrackedObstacle() {
    free_names_.push_back(obstacle_.obstacle_id);
  }

  void updateMeasurement(const CircleObstacle& new_obstacle) {
    kf_x_.y(0) = new_obstacle.center.x;
    kf_y_.y(0) = new_obstacle.center.y;
    kf_r_.y(0) = new_obstacle.radius;

    fade_counter_ = fade_counter_size_;
  }

  void updateTracking() {
    kf_x_.updateState();
    kf_y_.updateState();
    kf_r_.updateState();

    obstacle_.center.x = kf_x_.q_est(0);
    obstacle_.center.y = kf_y_.q_est(0);

    obstacle_.velocity.x = kf_x_.q_est(1);
    obstacle_.velocity.y = kf_y_.q_est(1);

    obstacle_.radius = kf_r_.q_est(0);

    fade_counter_--;
  }

  void setId(const std::string id) {
    obstacle_.obstacle_id = id;
  }

  static void setSamplingTime(double tp) {
    sampling_time_ = tp;
  }

  static void setCounterSize(int size) {
    fade_counter_size_ = size;
  }

  static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
    process_variance_ = process_var;
    process_rate_variance_ = process_rate_var;
    measurement_variance_ = measurement_var;
  }

  const CircleObstacle& getObstacle() const { return obstacle_; }

  const KalmanFilter& getKFx() const { return kf_x_; }
  const KalmanFilter& getKFy() const { return kf_y_; }
  const KalmanFilter& getKFr() const { return kf_r_; }

  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }

private:
  CircleObstacle obstacle_;
  KalmanFilter kf_x_;
  KalmanFilter kf_y_;
  KalmanFilter kf_r_;

  static int obstacle_number_;
  static std::list<std::string> free_names_;
  static int fade_counter_size_;

  static double sampling_time_;
  static double process_variance_;
  static double process_rate_variance_;
  static double measurement_variance_;

  int fade_counter_;
};

}
