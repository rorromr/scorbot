

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "scorbot_haptic_teleop/haptic_proxy_marker.h"

HapticProxyMarker::HapticProxyMarker(HapticProxyPtr proxy, std::string marker_topic, ros::NodeHandle nh):
    proxy_(proxy),
    nh_(nh)
{
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);
  ROS_DEBUG_STREAM("Publishing Haptic proxy markers on topic " << marker_pub_.getTopic());
}

void HapticProxyMarker::updateProxyMarker()
{
  // Get proxy_ position
  geometry_msgs::PoseStamped proxy_pose, hip_pose;
  proxy_->getProxyPosition(proxy_pose);
  proxy_->getHipPosition(hip_pose);
  // Sphere markers
  r1_marker_.type = visualization_msgs::Marker::SPHERE;
  r2_marker_.type = visualization_msgs::Marker::SPHERE;
  r3_marker_.type = visualization_msgs::Marker::SPHERE;
  state_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  normal_marker_.type = visualization_msgs::Marker::ARROW;
  force_marker_.type = visualization_msgs::Marker::ARROW;
  // Add option
  r1_marker_.action = visualization_msgs::Marker::ADD;
  r2_marker_.action = visualization_msgs::Marker::ADD;
  r3_marker_.action = visualization_msgs::Marker::ADD;
  state_marker_.action = visualization_msgs::Marker::ADD;
  normal_marker_.action = visualization_msgs::Marker::ADD;
  force_marker_.action = visualization_msgs::Marker::ADD;
  //  Update marker pose
  r1_marker_.pose.position = hip_pose.pose.position;
  r1_marker_.pose.orientation = hip_pose.pose.orientation;
  r2_marker_.pose.position = proxy_pose.pose.position;
  r3_marker_.pose.orientation = proxy_pose.pose.orientation;
  r3_marker_.pose.position = proxy_pose.pose.position;
  r3_marker_.pose.orientation = proxy_pose.pose.orientation;
  state_marker_.pose.position = proxy_pose.pose.position;
  // Updade normal marker
  geometry_msgs::Point normal_marker_start, normal_marker_end;
  normal_marker_start = proxy_pose.pose.position;
  Eigen::Vector3f normal;
  bool valid_normal = proxy_->getNormal(normal);
  normal *= 0.1;
  normal_marker_end.x = normal.x()+normal_marker_start.x;
  normal_marker_end.y = normal.y()+normal_marker_start.y;
  normal_marker_end.z = normal.z()+normal_marker_start.z;
  normal_marker_.points.clear();
  normal_marker_.points.push_back(normal_marker_start);
  normal_marker_.points.push_back(normal_marker_end);
  // Updade force marker
  geometry_msgs::Point force_marker_start, force_marker_end;
  force_marker_start = proxy_pose.pose.position;
  Eigen::Vector3f force;
  proxy_->getForce(force);
  force_marker_end.x = force.x()+force_marker_start.x;
  force_marker_end.y = force.y()+force_marker_start.y;
  force_marker_end.z = force.z()+force_marker_start.z;
  force_marker_.points.clear();
  force_marker_.points.push_back(force_marker_start);
  force_marker_.points.push_back(force_marker_end);

  // Set the scale of the markers
  double scale_1, scale_2, scale_3;
  proxy_->getRadious(scale_1, scale_2, scale_3);
  r1_marker_.scale.x = scale_1*2.0;
  r1_marker_.scale.y = scale_1*2.0;
  r1_marker_.scale.z = scale_1*2.0;
  r2_marker_.scale.x = scale_2*2.0;
  r2_marker_.scale.y = scale_2*2.0;
  r2_marker_.scale.z = scale_2*2.0;
  r3_marker_.scale.x = scale_3*2.0;
  r3_marker_.scale.y = scale_3*2.0;
  r3_marker_.scale.z = scale_3*2.0;
  normal_marker_.scale.x = 0.01;
  normal_marker_.scale.y = 0.02;
  normal_marker_.scale.z = 0.01;
  force_marker_.scale.x = 0.01;
  force_marker_.scale.y = 0.02;
  force_marker_.scale.z = 0.01;
  // Status text
  state_marker_.text = proxy_->getProxyStateName();
  state_marker_.scale.x = 0.010;
  state_marker_.scale.y = 0.020;
  state_marker_.scale.z = 0.015;
  state_marker_.pose.position.z += scale_3+0.05;
  // Set color for R1
  r1_marker_.color.r = 1.00f;
  r1_marker_.color.g = 0.34f;
  r1_marker_.color.b = 0.20f;
  r1_marker_.color.a = 1.00f;
  // Set color for R2
  r2_marker_.color.r = 1.00f;
  r2_marker_.color.g = 0.75f;
  r2_marker_.color.b = 0.00f;
  r2_marker_.color.a = 0.50f;
  // Set color for R3
  r3_marker_.color.r = 0.15f;
  r3_marker_.color.g = 0.68f;
  r3_marker_.color.b = 0.37f;
  r3_marker_.color.a = 0.50f;
  // Set color for state text
  state_marker_.color.r = 1.0f;
  state_marker_.color.g = 1.0f;
  state_marker_.color.b = 1.0f;
  state_marker_.color.a = 1.0f;
  // Set color force normal
  normal_marker_.color.r = 0.0f;
  normal_marker_.color.g = 0.0f;
  normal_marker_.color.b = 1.0f;
  normal_marker_.color.a = 1.0f;
  if (!valid_normal)
  {
    normal_marker_.action = visualization_msgs::Marker::DELETE;
    normal_marker_.color.a = 0.0f;
  }
  // Set color for force
  force_marker_.color.r = 1.0f;
  force_marker_.color.g = 0.0f;
  force_marker_.color.b = 0.0f;
  force_marker_.color.a = 1.0f;
  if (force.norm() < 0.001)
  {
    force_marker_.action = visualization_msgs::Marker::DELETE;
    force_marker_.color.a = 0.0f;
  }
  // Set id and namespace
  r1_marker_.id = 1;
  r1_marker_.ns = "proxy_marker/r1";
  r2_marker_.id = 2;
  r2_marker_.ns = "proxy_marker/r2";
  r3_marker_.id = 3;
  r3_marker_.ns = "proxy_marker/r3";
  state_marker_.id = 4;
  state_marker_.ns = "proxy_marker/state";
  normal_marker_.id = 5;
  normal_marker_.ns = "proxy_marker/normal";
  force_marker_.id = 6;
  force_marker_.ns = "proxy_marker/force";
  // Update header
  ros::Time now = ros::Time::now();
  r1_marker_.header.frame_id = proxy_pose.header.frame_id;
  r1_marker_.header.stamp = now;
  r2_marker_.header.frame_id = proxy_pose.header.frame_id;
  r2_marker_.header.stamp = now;
  r3_marker_.header.frame_id = proxy_pose.header.frame_id;
  r3_marker_.header.stamp = now;
  state_marker_.header.frame_id = proxy_pose.header.frame_id;
  state_marker_.header.stamp = now;
  normal_marker_.header.frame_id = proxy_pose.header.frame_id;
  normal_marker_.header.stamp = now;
  force_marker_.header.frame_id = proxy_pose.header.frame_id;
  force_marker_.header.stamp = now;
  // Publish Markers
  visualization_msgs::MarkerArray marker;
  marker.markers.push_back(r1_marker_);
  marker.markers.push_back(r2_marker_);
  marker.markers.push_back(r3_marker_);
  marker.markers.push_back(state_marker_);
  marker.markers.push_back(normal_marker_);
  marker.markers.push_back(force_marker_);
  marker_pub_.publish(marker);
}

