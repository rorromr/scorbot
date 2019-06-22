#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
//PCL///////////////////////////////////////
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
//Marker
#include <geometry_msgs/PoseStamped.h>
//Force
#include <geometry_msgs/WrenchStamped.h>
//Octree///////////////////////////////
#include <pcl/octree/octree.h>
//Voxelgrid//////////////////////////
#include <pcl/filters/voxel_grid.h>
//Mensaje callback////////////////////////
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
// TF
#include <tf/transform_listener.h>
//
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <math.h>
// Markers
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// OpenCV
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_types.h>
#include <eigen_conversions/eigen_msg.h>
#include <string>

//
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSolidPrimitive(const shape_msgs::SolidPrimitive& primitive, const geometry_msgs::Pose& pose, const double sample_size = 0.001)
{
  typedef  struct {
      double init;
      std::size_t N;
  } Range;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  switch (primitive.type)
  {
      case shape_msgs::SolidPrimitive::BOX:
      {
        // Range for X direction
        Range range_x;
        range_x.init = -0.5*primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X];
        range_x.N = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X]/sample_size;
        // Range for Y direction
        Range range_y;
        range_y.init = -0.5*primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
        range_y.N = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y]/sample_size;
        // Range for Z direction
        Range range_z;
        range_z.init = -0.5*primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
        range_z.N = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z]/sample_size;
        // Create point cloud
        std::size_t N = range_x.N + range_y.N + range_z.N;
        cloud_ptr->points.reserve(N);
        double x = range_x.init;
        double y = range_y.init;
        double z = range_z.init;
        // Fill the point cloud
        ROS_DEBUG_STREAM("init " << range_x.init << ", " << range_y.init << ", " << range_z.init);
        ROS_DEBUG_STREAM("N " << range_x.N << ", " << range_y.N << ", " << range_z.N);
        ROS_DEBUG_STREAM("Creating points");
        for (std::size_t i = 0; i < range_x.N; i++)
        {
          x += sample_size;
          for (std::size_t j = 0; j < range_y.N; j++)
          {
            y += sample_size;
            for (std::size_t k = 0; k < range_z.N; k++)
            {
              z += sample_size;
              pcl::PointXYZ point;
              point.x = x; point.y = y; point.z = z;
              cloud_ptr->points.push_back(point);
            }
            z = range_z.init;
          }
          y = range_y.init;
        }
        cloud_ptr->height = 1;
        cloud_ptr->width = (int) cloud_ptr->points.size();
        cloud_ptr->is_dense = true;
        break;
      }
      default:
        ROS_ERROR_STREAM("Sample algorithm not available for type " << primitive.type);
  }
  // Transform point cloud
  Eigen::Affine3d transform;
  tf::poseMsgToEigen(pose, transform);
  pcl::transformPointCloud(*cloud_ptr,*cloud_ptr,transform);
  return cloud_ptr;
}



class HapticProxy {
    // Radio
    double r1, r2, r3;
    // Octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree;
    pcl::PointXYZ proxy_position;
    // For store octree results
    std::vector<int> point_idx_r1;
    std::vector<int> point_idx_r2;
    std::vector<int> point_idx_r3;
    std::vector<float> point_distance_r1;
    std::vector<float> point_distance_r2;
    std::vector<float> point_distance_r3;

    std::string frame_id;

    struct ProxyState {
        typedef enum
        {
            FREE,
            CONTACT,
            ENTRENCHMENT,
            PC_FREE
        } Type;
    };

    ProxyState::Type state;
public:
    HapticProxy(const double r1, const double r2, const double r3, const std::string& frame_id)
    {
      double octree_resolution = 512.0;
      octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(octree_resolution));
      // Set initial position for Proxy
      proxy_position.x = 0.0f;
      proxy_position.y = 0.0f;
      proxy_position.z = 0.0f;
      // Set initial state as Free
      state = ProxyState::FREE;
      setRadious(r1, r2, r3);
      this->frame_id = frame_id;
    }

    void setRadious(const double r1, const double r2, const double r3)
    {
      this->r1 = r1;
      this->r2 = r2;
      this->r3 = r3;
    }

    void getRadious(double& r1, double& r2, double& r3) const
    {
      r1 = this->r1;
      r2 = this->r2;
      r3 = this->r3;
    }

    void updateOctreePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
      // Delete previous information and regenerate octree based on new point cloud
      octree->deleteTree();
      octree->setInputCloud(cloud);
      octree->addPointsFromInputCloud();
    }

    void setProxyPosition(const double x, const double y, const double z)
    {
      // Set position for Proxy
      proxy_position.x = x;
      proxy_position.y = y;
      proxy_position.z = z;
    }

    void getProxyPosition(geometry_msgs::PoseStamped& pose) const
    {
      pose.header.frame_id = frame_id;
      pose.pose.position.x = proxy_position.x;
      pose.pose.position.y = proxy_position.y;
      pose.pose.position.z = proxy_position.z;
      // Default orientation
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
    }

    void updateProxyPoints()
    {
      // Clear index arrays
      point_idx_r1.clear();
      point_idx_r2.clear();
      point_idx_r3.clear();
      point_distance_r1.clear();
      point_distance_r2.clear();
      point_distance_r3.clear();
      // Get values from octree
      octree->radiusSearch(proxy_position, r1, point_idx_r1, point_distance_r1);
      ROS_INFO_STREAM("point_idx_r1.size() = "  << point_idx_r1.size());
      octree->radiusSearch(proxy_position, r2, point_idx_r2, point_distance_r2);
      ROS_INFO_STREAM("point_idx_r2.size() = "  << point_idx_r2.size());
      octree->radiusSearch(proxy_position, r3, point_idx_r3, point_distance_r3);
      ROS_INFO_STREAM("point_idx_r3.size() = "  << point_idx_r3.size());
    }

    void uddateProxyStare(){
      // Update proxy state based on the amount of points in each sphere

      // Check for free state, no points in bigger sphere
      if(point_idx_r3.size() == 0)
        state = ProxyState::PC_FREE;
      else if(point_idx_r2.size() == 0)
        state = ProxyState::FREE;
      else if(point_idx_r2.size() != 0 && point_idx_r1.size() == 0)
        state = ProxyState::CONTACT;
      else if(point_idx_r1.size() != 0)
        state = ProxyState::ENTRENCHMENT;
      ROS_INFO_STREAM("Current status: " << getStateName());
    }

    std::string getStateName()
    {
      std::string state_name;
      switch (state)
      {
        case ProxyState::PC_FREE:
          state_name = "PC_FREE";
          break;
        case ProxyState::FREE:
          state_name = "FREE";
          break;
        case ProxyState::ENTRENCHMENT:
          state_name = "ENTRENCHMENT";
          break;
        case ProxyState::CONTACT:
          state_name = "CONTACT";
      }
      return state_name;
    }
};
typedef boost::shared_ptr<HapticProxy> HapticProxyPtr;


class HapticProxyMarker
{
    // Markers
    visualization_msgs::Marker r1_marker, r2_marker, r3_marker, state_marker;
    HapticProxyPtr proxy;
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
  public:
    HapticProxyMarker(HapticProxyPtr proxy, ros::NodeHandle node_handle):
      proxy(proxy),
      nh(node_handle)
    {
      marker_pub = nh.advertise<visualization_msgs::MarkerArray>("proxy_marker", 1);
    }

    void updateProxyMarker()
    {
      // Get proxy position
      geometry_msgs::PoseStamped proxy_pose;
      proxy->getProxyPosition(proxy_pose);
      // Sphere markers
      r1_marker.type = visualization_msgs::Marker::SPHERE;
      r2_marker.type = visualization_msgs::Marker::SPHERE;
      r3_marker.type = visualization_msgs::Marker::SPHERE;
      state_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      // Add option
      r1_marker.action = visualization_msgs::Marker::ADD;
      r2_marker.action = visualization_msgs::Marker::ADD;
      r3_marker.action = visualization_msgs::Marker::ADD;
      state_marker.action = visualization_msgs::Marker::ADD;
      //  Update marker pose
      r1_marker.pose.position = proxy_pose.pose.position;
      r1_marker.pose.orientation = proxy_pose.pose.orientation;
      r2_marker.pose.position = proxy_pose.pose.position;
      r3_marker.pose.orientation = proxy_pose.pose.orientation;
      r3_marker.pose.position = proxy_pose.pose.position;
      r3_marker.pose.orientation = proxy_pose.pose.orientation;
      state_marker.pose.position = proxy_pose.pose.position;

      // Set the scale of the markers
      double scale_1, scale_2, scale_3;
      proxy->getRadious(scale_1, scale_2, scale_3);
      r1_marker.scale.x = scale_1;
      r1_marker.scale.y = scale_1;
      r1_marker.scale.z = scale_1;
      r2_marker.scale.x = scale_2;
      r2_marker.scale.y = scale_2;
      r2_marker.scale.z = scale_2;
      r3_marker.scale.x = scale_3;
      r3_marker.scale.y = scale_3;
      r3_marker.scale.z = scale_3;
      // Status text
      state_marker.text = proxy->getStateName();
      state_marker.scale.x = 0.3;
      state_marker.scale.y = 0.3;
      state_marker.scale.z = 0.1;
      state_marker.pose.position.z += scale_3+0.05;
      // Set the color and alpha
      r1_marker.color.r = 1.00f;
      r1_marker.color.g = 0.34f;
      r1_marker.color.b = 0.20f;
      r1_marker.color.a = 1.00f;

      r2_marker.color.r = 1.00f;
      r2_marker.color.g = 0.75f;
      r2_marker.color.b = 0.00f;
      r2_marker.color.a = 0.50f;

      r3_marker.color.r = 0.15f;
      r3_marker.color.g = 0.68f;
      r3_marker.color.b = 0.37f;
      r3_marker.color.a = 0.50f;

      state_marker.color.r = 1.0f;
      state_marker.color.g = 1.0f;
      state_marker.color.b = 1.0f;
      state_marker.color.a = 1.0f;
      // Set id and namespace
      r1_marker.id = 1;
      r1_marker.ns = "proxy_marker/r1";
      r2_marker.id = 2;
      r2_marker.ns = "proxy_marker/r2";
      r3_marker.id = 3;
      r3_marker.ns = "proxy_marker/r3";
      state_marker.id = 4;
      state_marker.ns = "proxy_marker/state";
      // Update header
      ros::Time now = ros::Time::now();
      r1_marker.header.frame_id = proxy_pose.header.frame_id;
      r1_marker.header.stamp = now;
      r2_marker.header.frame_id = proxy_pose.header.frame_id;
      r2_marker.header.stamp = now;
      r3_marker.header.frame_id = proxy_pose.header.frame_id;
      r3_marker.header.stamp = now;
      state_marker.header.frame_id = proxy_pose.header.frame_id;
      state_marker.header.stamp = now;
      // Publish Markers
      visualization_msgs::MarkerArray marker;
      marker.markers.push_back(r1_marker);
      marker.markers.push_back(r2_marker);
      marker.markers.push_back(r3_marker);
      marker.markers.push_back(state_marker);
      marker_pub.publish(marker);
    }


};

int main (int argc, char** argv){
  shape_msgs::SolidPrimitive sample;
  sample.type  = shape_msgs::SolidPrimitive::BOX;
  sample.dimensions.push_back(0.1);
  sample.dimensions.push_back(0.2);
  sample.dimensions.push_back(0.05);

  // Initialize ROS
  ros::init (argc, argv, "proxy_test");
  ros::NodeHandle nh;
  ros::Publisher pub;
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  ros::Rate rate(30);

  HapticProxyPtr proxy = boost::make_shared<HapticProxy>(0.05, 0.075, 0.1, "world");
  HapticProxyMarker proxy_marker(proxy, nh);

  double roll = 0.0, pitch = 0, yaw = 0.0;
  ROS_INFO_STREAM("Starting publishing");
  while(ros::ok())
  {
    proxy_marker.updateProxyMarker();
    yaw += 0.1;
    Eigen::Quaterniond q;
    q =   Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ());
    Eigen::Affine3d pose = Eigen::Translation3d(0.0, 0.0, 0.0) * q;
    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(pose, pose_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sample_exmple_ptr = sampleSolidPrimitive(sample, pose_msg, 0.01);
    // Convert to ROS data type
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
    pcl::toROSMsg<pcl::PointXYZ>(*sample_exmple_ptr, *output);
    output->header.frame_id = "world";
    // Publish the data
    output->header.stamp = ros::Time::now();
    pub.publish(output);
    rate.sleep();
  }
}
