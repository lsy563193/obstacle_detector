/*
 This node converts Pose2D messages into TFs
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

geometry_msgs::Pose2D youbot_pose;

void myCallback(const geometry_msgs::Pose2D::ConstPtr& pose) {
  youbot_pose = *pose;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "opti_tf_publisher");
  ros::NodeHandle nh;

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose2D>("youbot_pose", 10, myCallback);

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(100.0);
  while (ros::ok()) {
    ros::spinOnce();

    transform.setOrigin(tf::Vector3(youbot_pose.x, youbot_pose.y, 0.0));

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, youbot_pose.theta);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base"));

    rate.sleep();
  }

  return 0;
}
