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

#include "../include/obstacle_visualizer.h"

using namespace obstacle_detector;

ObstacleVisualizer::ObstacleVisualizer() : nh_(""), nh_local_("~") {
  nh_local_.param("tracked_circles_color", p_tracked_circles_color_, 3);
  nh_local_.param("untracked_circles_color", p_untracked_circles_color_, 2);
  nh_local_.param("segments_color", p_segments_color_, 1);
  nh_local_.param("text_color", p_text_color_, 1);
  nh_local_.param("alpha", p_alpha_, 1.0);
  nh_local_.param("z_layer", p_z_layer_, 0.0);

  obstacles_sub_ = nh_.subscribe<obstacle_detector::Obstacles>("obstacles", 10, &ObstacleVisualizer::obstaclesCallback, this);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacles_markers", 10);

  setColor(tracked_circles_color_, p_tracked_circles_color_, p_alpha_);
  setColor(untracked_circles_color_, p_untracked_circles_color_, p_alpha_);
  setColor(segments_color_, p_segments_color_, p_alpha_);
  setColor(text_color_, p_text_color_, p_alpha_);

  ROS_INFO("Obstacle Visualizer [OK]");
  ros::spin();
}

void ObstacleVisualizer::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles) {
  visualization_msgs::MarkerArray markers_array;

  // Create markers for all of the circular obstacles
  visualization_msgs::Marker circle_marker;

  circle_marker.header.frame_id    = obstacles->header.frame_id;
  circle_marker.header.stamp       = obstacles->header.stamp;
  circle_marker.lifetime           = ros::Duration();
  circle_marker.id                 = 0;
  circle_marker.ns                 = "circles";
  circle_marker.type               = visualization_msgs::Marker::CYLINDER;
  circle_marker.action             = visualization_msgs::Marker::ADD;
  circle_marker.pose.position.z    = p_z_layer_;

  for (auto circle : obstacles->circles) {
    circle_marker.pose.position.x = circle.center.x;
    circle_marker.pose.position.y = circle.center.y;
    circle_marker.scale.x = 2.0 * circle.radius;
    circle_marker.scale.y = 2.0 * circle.radius;
    circle_marker.id++;

    if (circle.tracked)
      circle_marker.color = tracked_circles_color_;
    else
      circle_marker.color = untracked_circles_color_;

    markers_array.markers.push_back(circle_marker);
  }

  // Add some empty markers with DELETE action to get rid of the old ones
  circle_marker.action = visualization_msgs::Marker::DELETE;
  circle_marker.scale.x = 0.0;
  circle_marker.scale.y = 0.0;
  circle_marker.scale.z = 0.0;

  for (int i = 0; i < 20; ++i) {
    circle_marker.id++;
    markers_array.markers.push_back(circle_marker);
  }

  // Create text markers for all of the tracked obstacles
  visualization_msgs::Marker text_marker;

  text_marker.header.frame_id    = obstacles->header.frame_id;
  text_marker.header.stamp       = obstacles->header.stamp;
  text_marker.lifetime           = ros::Duration();
  text_marker.id                 = 0;
  text_marker.ns                 = "names";
  text_marker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action             = visualization_msgs::Marker::ADD;
  text_marker.pose.position.z    = p_z_layer_;
  text_marker.scale.z            = 0.2;

  for (auto circle : obstacles->circles) {
    if (circle.tracked) {
      text_marker.pose.position.x = circle.center.x;
      text_marker.pose.position.y = circle.center.y + circle.radius + 0.1;
      text_marker.id++;
      text_marker.color = text_color_;
      text_marker.text = "O" + std::to_string(circle.obstacle_id);

      markers_array.markers.push_back(text_marker);
    }
  }

  // Add some empty markers with DELETE action to get rid of the old ones
  text_marker.action = visualization_msgs::Marker::DELETE;
  text_marker.scale.x = 0.0;
  text_marker.scale.y = 0.0;
  text_marker.scale.z = 0.0;

  for (int i = 0; i < 20; ++i) {
    text_marker.id++;
    markers_array.markers.push_back(text_marker);
  }

  // Create markers for all of the segment obstacles
  visualization_msgs::Marker segments_marker;

  segments_marker.header.frame_id    = obstacles->header.frame_id;
  segments_marker.header.stamp       = obstacles->header.stamp;
  segments_marker.lifetime           = ros::Duration();
  segments_marker.id                 = 0;
  segments_marker.ns                 = "segments";
  segments_marker.type               = visualization_msgs::Marker::LINE_LIST;
  segments_marker.action             = visualization_msgs::Marker::ADD;
  segments_marker.pose.position.z    = p_z_layer_;
  segments_marker.pose.orientation.w = 1.0;
  segments_marker.scale.x            = 0.04;
  segments_marker.scale.y            = 0.04;
  segments_marker.scale.z            = 0.1;
  segments_marker.color              = segments_color_;

  for (auto segment : obstacles->segments) {
    segments_marker.points.push_back(segment.first_point);
    segments_marker.points.push_back(segment.last_point);
  }

  markers_array.markers.push_back(segments_marker);

  markers_pub_.publish(markers_array);
}

void ObstacleVisualizer::setColor(std_msgs::ColorRGBA &color, int color_code, float alpha) {
  switch (color_code) {
  case 0:
    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.0;
    break;
  case 1:
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    break;
  case 2:
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    break;
  case 3:
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    break;
  case 4:
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    break;
  case 5:
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;
    break;
  case 6:
    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;
    break;
  case 7:
    color.r = 0.0;
    color.g = 1.0;
    color.b = 1.0;
    break;
  }

  color.a = alpha;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_visualizer");
  ObstacleVisualizer ov;
  return 0;
}
