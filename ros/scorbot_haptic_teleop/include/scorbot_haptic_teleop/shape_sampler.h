#ifndef SHAPE_SAMPLER_H
#define SHAPE_SAMPLER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSolidPrimitive(const shape_msgs::SolidPrimitive& primitive,
    const geometry_msgs::Pose& pose, const std::string& frame_id, const double sample_size = 0.001);

#endif //SHAPE_SAMPLER_H
