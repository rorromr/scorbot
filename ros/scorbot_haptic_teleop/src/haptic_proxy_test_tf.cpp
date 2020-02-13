#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <omni_msgs/OmniFeedback.h>


#include "scorbot_haptic_teleop/haptic_proxy.h"
#include "scorbot_haptic_teleop/haptic_proxy_marker.h"
#include "scorbot_haptic_teleop/shape_sampler.h"
#include "scorbot_haptic_teleop/haptic_proxy_reconfigure.h"


int main (int argc, char** argv){
  shape_msgs::SolidPrimitive sample;
  sample.type  = shape_msgs::SolidPrimitive::BOX;
  sample.dimensions.push_back(0.6);
  sample.dimensions.push_back(0.6);
  sample.dimensions.push_back(0.01);

  // Initialize ROS
  ros::init (argc, argv, "proxy_test");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  // Haptic force
  ros::Publisher haptic_force_pub = nh.advertise<omni_msgs::OmniFeedback>("/omni/force_feedback", 10);
  ros::Rate rate(30);

  geometry_msgs::PoseStamped target_hip_pose;
  target_hip_pose.pose.position.x = 0.0;
  target_hip_pose.pose.position.y = 0.0;
  target_hip_pose.pose.position.z = 0.1;
  target_hip_pose.pose.orientation.x = 0.0;
  target_hip_pose.pose.orientation.y = 0.0;
  target_hip_pose.pose.orientation.z = 0.0;
  target_hip_pose.pose.orientation.w = 1.0;
  target_hip_pose.header.frame_id = "world";

  HapticProxyPtr proxy = boost::make_shared<HapticProxy>(0.005, 0.010, 0.015, 60.0, "world");
  HapticProxyReconfig proxy_reconf(proxy, nh);
  HapticProxyMarker proxy_marker(proxy, "/proxy_marker", nh);
  // Override init position of the Proxy
  proxy->setProxyPositionOverride(0.1, 0.0, 0.08);

  double roll = 0.0, pitch = 0, yaw = 0.0;
  ROS_INFO_STREAM("Starting publishing");

  tf::TransformListener tf_listener;

  while(ros::ok())
  {
    yaw += 0.01;
    Eigen::Quaterniond q;
    q =   Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ());
    Eigen::Affine3d sample_pose = Eigen::Translation3d(0.0, 0.0, 0.06) * q;
    geometry_msgs::Pose sample_pose_msg;
    tf::poseEigenToMsg(sample_pose, sample_pose_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sample_exmple_ptr = sampleSolidPrimitive(sample, sample_pose_msg, "world", 0.005);
    // Convert to ROS data type
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
    pcl::toROSMsg<pcl::PointXYZ>(*sample_exmple_ptr, *output);
    // Publish the data
    output->header.stamp = ros::Time::now();
    pub.publish(output);

    // Update proxy_ point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_proxy(new pcl::PointCloud<pcl::PointXYZ>());
    bool transform_point_cloud = pcl_ros::transformPointCloud<pcl::PointXYZ>(proxy->getFrameId(),*sample_exmple_ptr, *point_cloud_proxy, tf_listener);

    // Get target position from TF
    tf::StampedTransform transform;
    try
    {
      tf_listener.lookupTransform("/world", "/omni/stylus", ros::Time(0), transform);
      target_hip_pose.pose.position.x = transform.getOrigin().getX();
      target_hip_pose.pose.position.y = transform.getOrigin().getY();
      target_hip_pose.pose.position.z = transform.getOrigin().getZ();
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      rate.sleep();
      continue;
    }

    if (transform_point_cloud)
    {
      proxy->updatePointCloud(point_cloud_proxy);
      proxy->updateProxyPoints();
      proxy->updateProxyState();
      proxy->setHipPosition(target_hip_pose.pose.position.x, target_hip_pose.pose.position.y, target_hip_pose.pose.position.z);
      if (proxy->getProxyState() != HapticProxy::ProxyState::PC_FREE)
      {
        Eigen::Vector3f normal;
        bool normal_validation = proxy->updateNormal();
      }
    }
    proxy_marker.updateProxyMarker();
    // Update force
    Eigen::Vector3f force;
    proxy->getForce(force);
    omni_msgs::OmniFeedback force_cmd_;
    force_cmd_.force.x = -force.x();
    force_cmd_.force.z = force.y();
    force_cmd_.force.y = force.z();
    haptic_force_pub.publish(force_cmd_);

    ros::spinOnce();
    rate.sleep();
  }
}
