﻿/*
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

#include "../include/obstacle_recorder.h"

using namespace obstacle_detector;
using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_recorder");
  ObstacleRecorder OR;
  return 0;
}

ObstacleRecorder::ObstacleRecorder() : nh_(""), nh_local_("~"), p_active_(false), p_recording_(false) {
  std_srvs::Empty empty;
  updateParams(empty.request, empty.response);
  params_srv_ = nh_local_.advertiseService("params", &ObstacleRecorder::updateParams, this);

  ros::spin();
}

bool ObstacleRecorder::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;
  bool prev_recording = p_recording_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("recording", p_recording_, false);

  nh_local_.param<double>("min_x_limit", p_min_x_limit_, -10.0);
  nh_local_.param<double>("max_x_limit", p_max_x_limit_,  10.0);
  nh_local_.param<double>("min_y_limit", p_min_y_limit_, -10.0);
  nh_local_.param<double>("max_y_limit", p_max_y_limit_,  10.0);

  nh_local_.param<string>("filename_prefix", p_filename_prefix_, "tracked_");

  if (p_active_ != prev_active) {
    if (p_active_) {
      obstacles_sub_ = nh_.subscribe<Obstacles>("obstacles", 10, &ObstacleRecorder::obstaclesCallback, this);
      ROS_INFO("Obstacle Recorder [ACTIVE]");
    }
    else {
      obstacles_sub_.shutdown();
      nh_local_.setParam("recording", false);
      p_recording_ = false;
      ROS_INFO("Obstacle Recorder [OFF]");
    }
  }

  if (p_recording_ != prev_recording) {
    if (p_recording_) {
      prepareFile();
      ROS_INFO("Obstacle Recorder [RECORDING]");
    }
    else {
      file_.close();
      ROS_INFO("Obstacle Recorder [STOPPED RECORDING]");
    }
  }

  return true;
}

void ObstacleRecorder::obstaclesCallback(const Obstacles::ConstPtr& obstacles) {
  if (p_recording_) {
    counter_++;
    double t = (ros::Time::now() - start_mark_).toSec();

    for (auto circle : obstacles->circles) {
      if (circle.center.x < p_max_x_limit_ && circle.center.x > p_min_x_limit_ &&
          circle.center.y < p_max_y_limit_ && circle.center.y > p_min_y_limit_) {

      file_ << counter_ << "\t"
            << t << "\t"
            << (int)circle.tracked << "\t"
            << circle.center.x << "\t"
            << circle.center.y << "\t"
            << circle.radius << "\t"
            << circle.velocity.x << "\t"
            << 0 << "\n"; //circle.obstacle_id << "\n";
      }
    }
  }
}

void ObstacleRecorder::prepareFile() {
  time_t now = time(NULL);
  char the_date[30];
  start_mark_ = ros::Time::now();
  counter_ = 0;

  if (now != -1)
    strftime(the_date, 30, "%Y_%m_%d_%H_%M_%S", gmtime(&now));

  std::string username = getenv("USER");
  std::string foldername = "/home/" + username + "/ObstacleDetector/records/";
  std::string filename = foldername + p_filename_prefix_ + "obstacles_" + std::string(the_date) + ".txt";

  boost::filesystem::create_directories(foldername);
  file_.open(filename);

  // Number, time, tracked, x, y, r, v_x, v_y, label
  file_ << "ROS time at start: " << ros::Time::now() << "\n";
  file_ << "idx \t t \t tracked \t x \t y \t r \t x_p \t y_p \t label \n";
}
