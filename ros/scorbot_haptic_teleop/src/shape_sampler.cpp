
#include "scorbot_haptic_teleop/shape_sampler.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <eigen_conversions/eigen_msg.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSolidPrimitive(const shape_msgs::SolidPrimitive& primitive,
    const geometry_msgs::Pose& pose, const std::string& frame_id, const double sample_size)
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
      cloud_ptr->header.frame_id = frame_id;
      pcl_conversions::toPCL(ros::Time::now(), cloud_ptr->header.stamp);
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

