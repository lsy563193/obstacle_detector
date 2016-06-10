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

#include "../include/scans_merger.h"

using namespace obstacle_detector;

ScansMerger::ScansMerger() : nh_(""), nh_local_("~") {
  nh_local_.param<std::string>("world_frame", p_world_frame_, "world");
  nh_local_.param<std::string>("base_frame", p_base_frame_, "base");
  nh_local_.param<std::string>("front_frame", p_front_frame_, "front_scanner");
  nh_local_.param<std::string>("rear_frame", p_rear_frame_, "rear_scanner");

  nh_local_.param<int>("max_unreceived_scans", p_max_unreceived_scans_, 1);

  nh_local_.param<bool>("omit_overlapping_scans", p_omit_overlapping_scans_, false);
  nh_local_.param<bool>("transform_to_world", p_transform_to_world, true);

  nh_local_.param<double>("max_scanner_range", p_max_scanner_range_, 6.0);
  nh_local_.param<double>("max_x_range", p_max_x_range_, 2.0);
  nh_local_.param<double>("min_x_range", p_min_x_range_, -2.0);
  nh_local_.param<double>("max_y_range", p_max_y_range_, 2.0);
  nh_local_.param<double>("min_y_range", p_min_y_range_, -2.0);

  front_scan_sub_ = nh_.subscribe("front_scan", 10, &ScansMerger::frontScanCallback, this);
  rear_scan_sub_ = nh_.subscribe("rear_scan", 10, &ScansMerger::rearScanCallback, this);
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pcl", 10);

  try {
    front_tf_.waitForTransform(p_base_frame_, p_front_frame_, ros::Time::now(), ros::Duration(5.0));
    rear_tf_.waitForTransform(p_base_frame_, p_rear_frame_, ros::Time::now(), ros::Duration(5.0));
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
  }

  if (p_transform_to_world) {
    try {
      world_tf_.waitForTransform(p_world_frame_, p_base_frame_, ros::Time::now(), ros::Duration(5.0));
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
  }

  first_scan_received_ = false;
  second_scan_received_ = false;
  unreceived_front_scans_ = 0;
  unreceived_rear_scans_ = 0;

  ROS_INFO("Scans Merger [OK]");
  ros::spin();
}

void ScansMerger::frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& front_scan) {
  try {
    tf::StampedTransform transform;
    front_tf_.lookupTransform(p_base_frame_, p_front_frame_, ros::Time(0), transform);

    tf::Vector3 origin = transform.getOrigin();
    double yaw = tf::getYaw(transform.getRotation());

    geometry_msgs::Point32 local_point, base_point;
    float phi = front_scan->angle_min;

    for (const float r : front_scan->ranges) {
      if (r > front_scan->range_min && r < front_scan->range_max && r <= p_max_scanner_range_) {
        local_point.x = r * cos(phi);
        local_point.y = r * sin(phi);

        base_point = transformPoint(local_point, transform);

        if (!(p_omit_overlapping_scans_ && base_point.x < 0.0))
          pcl_msg_.points.push_back(base_point);
      }
      phi += front_scan->angle_increment;
    }

    first_scan_received_ = true;

    if (second_scan_received_ || unreceived_rear_scans_ > p_max_unreceived_scans_) {
      publishPCL();

      unreceived_front_scans_ = 0;
    }
    else unreceived_rear_scans_++;
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }
}

void ScansMerger::rearScanCallback(const sensor_msgs::LaserScan::ConstPtr& rear_scan) {
  try {
    tf::StampedTransform transform;
    rear_tf_.lookupTransform(p_base_frame_, p_rear_frame_, ros::Time(0), transform);

    tf::Vector3 origin = transform.getOrigin();
    double yaw = tf::getYaw(transform.getRotation());

    geometry_msgs::Point32 local_point, base_point;
    float phi = rear_scan->angle_min;

    for (const float r : rear_scan->ranges) {
      if (r > rear_scan->range_min && r < rear_scan->range_max && r <= p_max_scanner_range_) {
        local_point.x = r * cos(phi);
        local_point.y = r * sin(phi);

        base_point = transformPoint(local_point, transform);

        if (!(p_omit_overlapping_scans_ && base_point.x > 0.0))
          pcl_msg_.points.push_back(base_point);
      }
      phi += rear_scan->angle_increment;
    }

    second_scan_received_ = true;

    if (first_scan_received_ || unreceived_front_scans_ > p_max_unreceived_scans_) {
      publishPCL();

      unreceived_rear_scans_ = 0;
    }
    else unreceived_front_scans_++;
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }
}

geometry_msgs::Point32 ScansMerger::transformPoint(const geometry_msgs::Point32& p, const tf::StampedTransform& transform) {
  geometry_msgs::Point32 point;

  tf::Vector3 origin = transform.getOrigin();
  double yaw = tf::getYaw(transform.getRotation());

  point.x = p.x * cos(yaw) - p.y * sin(yaw) + origin.x();
  point.y = p.x * sin(yaw) + p.y * cos(yaw) + origin.y();

  return point;
}

bool ScansMerger::checkPointInLimits(const geometry_msgs::Point32& p) {
  if ((p.x > p_max_x_range_) || (p.x < p_min_x_range_) || (p.y > p_max_y_range_) || (p.y < p_min_y_range_))
    return false;
  else
    return true;
}

void ScansMerger::publishPCL() {
  if (p_transform_to_world) {
    try {
      tf::StampedTransform transform;
      world_tf_.lookupTransform(p_world_frame_, p_base_frame_, ros::Time(0), transform);

      sensor_msgs::PointCloud world_pcl_msg;
      geometry_msgs::Point32 world_point;

      for (auto point : pcl_msg_.points) {
        world_point = transformPoint(point, transform);

        if (checkPointInLimits(world_point))
          world_pcl_msg.points.push_back(world_point);
      }

      world_pcl_msg.header.frame_id = p_world_frame_;
      world_pcl_msg.header.stamp = ros::Time::now();
      pcl_pub_.publish(world_pcl_msg);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
  }
  else {
    pcl_msg_.header.frame_id = p_base_frame_;
    pcl_msg_.header.stamp = ros::Time::now();
    pcl_pub_.publish(pcl_msg_);
  }

  pcl_msg_.points.clear();

  first_scan_received_ = false;
  second_scan_received_ = false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scans_merger");
  ScansMerger SM;
  return 0;
}
