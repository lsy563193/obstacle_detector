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

ObstacleDetector::ObstacleDetector() : nh_(""), nh_local_("~") {
  nh_local_.param<string>("frame_id", p_frame_id_, "world");

  nh_local_.param<bool>("use_scan", p_use_scan_, true);
  nh_local_.param<bool>("use_pcl", p_use_pcl_, false);
  nh_local_.param<bool>("use_split_and_merge", p_use_split_and_merge_, false);
  nh_local_.param<bool>("discard_converted_segments", p_discard_converted_segments_, true);
  nh_local_.param<bool>("transform_coordinates", p_transform_coordinates_, true);

  nh_local_.param<int>("min_group_points", p_min_group_points_, 5);

  nh_local_.param<double>("max_group_distance", p_max_group_distance_, 0.100);
  nh_local_.param<double>("distance_proportion", p_distance_proportion_, 0.006136);
  nh_local_.param<double>("max_split_distance", p_max_split_distance_, 0.100);
  nh_local_.param<double>("max_merge_separation", p_max_merge_separation_, 0.100);
  nh_local_.param<double>("max_merge_spread", p_max_merge_spread_, 0.070);
  nh_local_.param<double>("max_circle_radius", p_max_circle_radius_, 0.200);
  nh_local_.param<double>("radius_enlargement", p_radius_enlargement_, 0.020);

  if (p_use_scan_)
    scan_sub_ = nh_.subscribe("scan", 10, &ObstacleDetector::scanCallback, this);
  else if (p_use_pcl_)
    pcl_sub_ = nh_.subscribe("pcl", 10, &ObstacleDetector::pclCallback, this);

  obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("obstacles", 10);

  ROS_INFO("Obstacle Detector [OK]");
  ros::spin();
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

  groupPointsAndDetectSegments();
  mergeSegments();

//  detectCircles();
//  mergeCircles();

  publishObstacles();

  input_points_.clear();
}

void ObstacleDetector::groupPointsAndDetectSegments() {
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

void ObstacleDetector::detectSegments(PointSet& point_set) {
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

      if (compareAndMergeSegments(*i, *j, merged_segment)) {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleDetector::compareAndMergeSegments(Segment& s1, Segment& s2, Segment& merged_s) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareAndMergeSegments(s2, s1, merged_s);

  /*
    Point s2_middle_point = (s2.first_point() + s2.last_point()) / 2.0;
    Point a = s1.last_point() - s1.first_point();
    Point b = s2_middle_point - s1.first_point();

    float t = a.dot(b) / a.lengthSquared();
    Point projection = s1.first_point() + t * a;    // Projection of s2 middle point onto s1

    || // Small separation ----  ----
    || // Full or partial occlusion ---====
      a.dot(s2.first_point() - s1.first_point()) * (-a).dot(s2.last_point() - s1.last_point()) < 0.0)
      (s1.first_point().cross(s2.first_point()) * s1.last_point().cross(s2.last_point()) < 0.0)
      (s2_middle_point - projection).lengthSquared() < pow(p_max_merge_separation_, 2.0) ||
      (s2.last_point() - s1.last_point()).lengthSquared() < pow(p_max_merge_separation_, 2.0) ||
      (s2.first_point() - s1.first_point()).lengthSquared() < pow(p_max_merge_separation_, 2.0))
  */

  if ((s1.last_point - s2.first_point).lengthSquared() < pow(p_max_merge_separation_, 2.0)) {

    vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = fitSegment(point_sets);

    if (segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
        segment.distanceTo(s1.last_point)  < p_max_merge_spread_ &&
        segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
        segment.distanceTo(s2.last_point)  < p_max_merge_spread_) {
      merged_s = segment;
      return true;
    }
  }

  return false;
}

void ObstacleDetector::detectCircles() {
//  for (auto itr = segments_.begin(); itr != segments_.end(); ++itr) {
//    Circle c(*itr);
//    c.setRadius(c.radius() + p_radius_enlargement_);

//    if (c.radius() < p_max_circle_radius_) {
//      c.point_set().assign(itr->point_set().begin(), itr->point_set().end());
//      circles_.push_back(c);

//      if (p_discard_converted_segments_) {
//        itr = segments_.erase(itr);
//        --itr;
//      }
//    }
//  }
}

void ObstacleDetector::mergeCircles() {
//  bool merged = false;
//  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
//    if (merged) {
//      i--;
//      merged = false;
//    }

//    auto j = i;
//    j++;
//    for (j; j != circles_.end(); ++j) {
//      if (compareAndMergeCircles(*i, *j)) {      // If merged - a new circle appeared at the end of the list
//        auto temp_ptr = i;
//        i = circles_.insert(i, circles_.back()); // i now points to new circle
//        circles_.pop_back();
//        circles_.erase(temp_ptr);
//        circles_.erase(j);

//        merged = true;
//        break;
//      }
//    }
//  }
}

bool ObstacleDetector::compareAndMergeCircles(Circle& c1, Circle& c2) {
//  // If circle c1 is fully inside c2 - merge and leave as c2
//  if (c1.radius() + (c2.center() - c1.center()).length() <= c2.radius()) {
//    circles_.push_back(c2);
//    return true;
//  }

//  // If circle c2 is fully inside c1 - merge and leave as c1
//  if (c2.radius() + (c2.center() - c1.center()).length() <= c1.radius()) {
//    circles_.push_back(c1);
//    return true;
//  }

//  // If circles intersect and are 'small' - merge
//  if (c1.radius() + c2.radius() >= (c2.center() - c1.center()).length()) {
//    Segment s(c1.center(), c2.center());
//    Circle c(s);
//    c.setRadius(c.radius() + max(c1.radius(), c2.radius()));

//    if (c.radius() < p_max_circle_radius_) {
//      c.point_set().assign(c1.point_set().begin(), c1.point_set().end());
//      c.point_set().insert(c.point_set().end(), c2.point_set().begin(), c2.point_set().end());
//      circles_.push_back(c);
//      return true;
//    }
//  }

//  return false;
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detector");
  ObstacleDetector od;
  return 0;
}
