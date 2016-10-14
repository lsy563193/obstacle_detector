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

#include "../include/obstacle_detector.h"

using namespace std;
using namespace obstacle_detector;

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detector");
  ObstacleDetector od;
  return 0;
}

ObstacleDetector::ObstacleDetector() : nh_(""), nh_local_("~"), p_active_(false) {
  std_srvs::Empty empty;
  updateParams(empty.request, empty.response);
  params_srv_ = nh_local_.advertiseService("params", &ObstacleDetector::updateParams, this);

  ros::spin();
}

bool ObstacleDetector::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<int>("min_group_points", p_min_group_points_, 5);

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("use_scan", p_use_scan_, true);
  nh_local_.param<bool>("use_pcl", p_use_pcl_, false);
  nh_local_.param<bool>("use_split_and_merge", p_use_split_and_merge_, false);
  nh_local_.param<bool>("discard_converted_segments", p_discard_converted_segments_, true);
  nh_local_.param<bool>("transform_coordinates", p_transform_coordinates_, true);

  nh_local_.param<double>("max_group_distance", p_max_group_distance_, 0.100);
  nh_local_.param<double>("distance_proportion", p_distance_proportion_, 0.006136);
  nh_local_.param<double>("max_split_distance", p_max_split_distance_, 0.070);
  nh_local_.param<double>("max_merge_separation", p_max_merge_separation_, 0.150);
  nh_local_.param<double>("max_merge_spread", p_max_merge_spread_, 0.070);
  nh_local_.param<double>("max_circle_radius", p_max_circle_radius_, 0.300);
  nh_local_.param<double>("radius_enlargement", p_radius_enlargement_, 0.030);

  nh_local_.param<string>("frame_id", p_frame_id_, "world");

  if (p_active_ != prev_active) {
    if (p_active_) {
      if (p_use_scan_)
        scan_sub_ = nh_.subscribe("scan", 10, &ObstacleDetector::scanCallback, this);
      else if (p_use_pcl_)
        pcl_sub_ = nh_.subscribe("pcl", 10, &ObstacleDetector::pclCallback, this);

      obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("obstacles", 10);

      ROS_INFO("Obstacle Detector [START]");
    }
    else {
      scan_sub_.shutdown();
      pcl_sub_.shutdown();
      obstacles_pub_.shutdown();

      ROS_INFO("Obstacle Detector [STOP]");
    }
  }

  return true;
}

void ObstacleDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  base_frame_id_ = scan->header.frame_id;

  double phi = scan->angle_min;

  for (const float r : scan->ranges) {
    if (r >= scan->range_min && r <= scan->range_max)
      input_points_.push_back(Point::fromPoolarCoords(r, phi));

    phi += scan->angle_increment;
  }

  processPoints();
}

void ObstacleDetector::pclCallback(const sensor_msgs::PointCloud::ConstPtr& pcl) {
  base_frame_id_ = pcl->header.frame_id;

  for (const geometry_msgs::Point32& point : pcl->points)
    input_points_.push_back(Point(point.x, point.y));

  processPoints();
}

void ObstacleDetector::processPoints() {
  segments_.clear();
  circles_.clear();

  groupPoints();  // Grouping points simultaneously detects segments
  mergeSegments();

  detectCircles();
  mergeCircles();

  publishObstacles();

  input_points_.clear();
}

void ObstacleDetector::groupPoints() {
  PointSet point_set;

  for (PointIterator point = input_points_.begin(); point != input_points_.end(); ++point) {
    if (point_set.num_points != 0) {
      double r = (*point).length();

      if ((*point - *point_set.end).lengthSquared() > pow(p_max_group_distance_ + r * p_distance_proportion_, 2.0)) {
        detectSegments(point_set);
        point_set.num_points = -1;
        advance(point, -1);
      }
      else
        point_set.end = point;
    }
    else {
      point_set.begin = point;
      point_set.end = point;
    }

    point_set.num_points++;
  }

  detectSegments(point_set); // Check the last point set too!
}

void ObstacleDetector::detectSegments(const PointSet& point_set) {
  if (point_set.num_points < p_min_group_points_)
    return;

  Segment segment(*point_set.begin, *point_set.end);  // Use Iterative End Point Fit

  if (p_use_split_and_merge_)
    segment = fitSegment(point_set);

  PointIterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  int split_index = 0; // Natural index of splitting point (counting from 1)
  int point_index = 0; // Natural index of current point in the set

  // Seek the point of division
  for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
    ++point_index;

    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      double r = (*point).length();

      if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
        max_distance = distance;
        set_divider = point;
        split_index = point_index;
      }
    }
  }

  // Split the set only if the sub-groups are not 'small'
  if (max_distance > 0.0 && split_index > p_min_group_points_ && split_index < point_set.num_points - p_min_group_points_) {
    set_divider = input_points_.insert(set_divider, *set_divider);  // Clone the dividing point for each group

    PointSet subset1, subset2;
    subset1.begin = point_set.begin;
    subset1.end = set_divider;
    subset1.num_points = split_index;

    subset2.begin = ++set_divider;
    subset2.end = point_set.end;
    subset2.num_points = point_set.num_points - split_index;

    detectSegments(subset1);
    detectSegments(subset2);
  } else {  // Add the segment
    if (!p_use_split_and_merge_)
      segment = fitSegment(point_set);

    segments_.push_back(segment);
  }
}

void ObstacleDetector::mergeSegments() {
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    for (auto j = i; j != segments_.end(); ++j) {
      Segment merged_segment;

      if (compareSegments(*i, *j, merged_segment)) {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleDetector::compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  // 1. Check if the segments are close to each other
  if (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
      s1.trueDistanceTo(s2.last_point)  < p_max_merge_separation_ ||
      s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
      s2.trueDistanceTo(s1.last_point)  < p_max_merge_separation_)
  {
    // 2. Check spread about line from regression of sum of points
    vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = fitSegment(point_sets);

    if (segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
        segment.distanceTo(s1.last_point)  < p_max_merge_spread_ &&
        segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
        segment.distanceTo(s2.last_point)  < p_max_merge_spread_)
    {
      merged_segment = segment;
      return true;
    }
  }

  return false;
}

void ObstacleDetector::detectCircles() {
  for (auto segment = segments_.begin(); segment != segments_.end(); ++segment) {
    Circle circle(*segment);
    circle.radius += p_radius_enlargement_;

    if (circle.radius < p_max_circle_radius_) {
      circles_.push_back(circle);

      if (p_discard_converted_segments_) {
        segment = segments_.erase(segment);
        --segment;
      }
    }
  }
}

void ObstacleDetector::mergeCircles() {
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    for (auto j = i; j != circles_.end(); ++j) {
      Circle merged_circle;

      if (compareCircles(*i, *j, merged_circle)) {
        auto temp_itr = circles_.insert(i, merged_circle);
        circles_.erase(i);
        circles_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleDetector::compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle) {
  if (&c1 == &c2)
    return false;

  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    merged_circle = c2;
    return true;
  }

  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    merged_circle = c1;
    return true;
  }

  // If circles intersect and are 'small' - merge
  if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
    Point center = c1.center + (c2.center - c1.center) * c1.radius / (c1.radius + c2.radius);
    double radius = (c1.center - center).length() + c1.radius;

    Circle circle(center, radius);
    circle.radius += max(c1.radius, c2.radius);

    if (circle.radius < p_max_circle_radius_) {
      circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(), c1.point_sets.end());
      circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(), c2.point_sets.end());
      merged_circle = circle;
      return true;
    }
  }

  return false;
}

void ObstacleDetector::publishObstacles() {
  Obstacles obstacles;
  obstacles.header.stamp = ros::Time::now();

  for (const Segment& s : segments_) {
    obstacle_detector::SegmentObstacle segment;

    segment.first_point.x = s.first_point.x;
    segment.first_point.y = s.first_point.y;
    segment.last_point.x = s.last_point.x;
    segment.last_point.y = s.last_point.y;

    obstacles.segments.push_back(segment);
  }

  for (const Circle& c : circles_) {
    obstacle_detector::CircleObstacle circle;

    circle.center.x = c.center.x;
    circle.center.y = c.center.y;
    circle.velocity.x = 0.0;
    circle.velocity.y = 0.0;
    circle.radius = c.radius;

    circle.obstacle_id = string("");
    circle.tracked = false;

    obstacles.circles.push_back(circle);
  }

  if (p_transform_coordinates_) {
    tf::StampedTransform transform;

    try {
      tf_listener_.lookupTransform(p_frame_id_, base_frame_id_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return;
    }

    tf::Vector3 origin = transform.getOrigin();
    double theta = tf::getYaw(transform.getRotation());

    for (auto& s : obstacles.segments) {
      s.first_point = transformPoint(s.first_point, origin.x(), origin.y(), theta);
      s.last_point = transformPoint(s.last_point, origin.x(), origin.y(), theta);
    }

    for (auto& c : obstacles.circles)
      c.center = transformPoint(c.center, origin.x(), origin.y(), theta);

    obstacles.header.frame_id = p_frame_id_;
  }
  else
    obstacles.header.frame_id = base_frame_id_;

  obstacles_pub_.publish(obstacles);
}
