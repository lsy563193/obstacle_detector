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

#include "../include/obstacle_recorder.h"

using namespace obstacle_detector;

ObstacleRecorder::ObstacleRecorder() : nh_(""), nh_local_("~") {
  nh_local_.param<std::string>("filename_prefix", p_filename_prefix_, "raw_");

  obstacles_sub_ = nh_.subscribe<obstacle_detector::Obstacles>("obstacles", 10, &ObstacleRecorder::obstaclesCallback, this);
  recording_trigger_srv_ = nh_.advertiseService(p_filename_prefix_ + "recording_trigger", &ObstacleRecorder::recordingTrigger, this);

  recording_ = false;

  ROS_INFO("Obstacle Recorder [OK]");
  ros::spin();
}

void ObstacleRecorder::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles) {
  if (recording_) {
    counter_++;
    float t = (ros::Time::now() - start_mark_).toSec();

    for (auto circle : obstacles->circles) {
      file_ << counter_ << "\t"
            << t << "\t"
            << circle.obstacle_id << "\t"
            << (int)circle.tracked << "\t"
            << circle.num_points << "\t"
            << circle.center.x << "\t"
            << circle.center.y << "\t"
            << circle.radius << "\t"
            << circle.velocity.x << "\t"
            << circle.velocity.y << "\n";
    }
  }
}

bool ObstacleRecorder::recordingTrigger(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!recording_) {
    time_t now = time(NULL);
    char the_date[30];
    start_mark_ = ros::Time::now();
    counter_ = 0;

    if (now != -1)
      strftime(the_date, 30, "%Y_%m_%d_%H_%M_%S", gmtime(&now));

    std::string username = getenv("USER");
    std::string foldername = "/home/" + username + "/obstacle_records/";
    std::string filename = foldername + p_filename_prefix_ + "obstacles_" + std::string(the_date) + ".txt";

    boost::filesystem::create_directories(foldername);
    file_.open(filename);

    // Number, time, label, tracked, x, y, r, v_x, v_y
    file_ << "ROS time at start: " << ros::Time::now() << "\n";
    file_ << "idx" << "\t" << "t" << "\t" << "label" << "\t" << "tracked" << "\t" << "N" << "\t" << "x" << "\t" << "y" << "\t" << "r" << "\t" << "x_p" << "\t" << "y_p" << "\n";

    recording_ = true;
  }
  else {
    file_.close();
    recording_ = false;
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_recorder");
  ObstacleRecorder obstacle_recorder;
  return 0;
}
