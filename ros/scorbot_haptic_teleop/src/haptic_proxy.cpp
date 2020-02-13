
#include "scorbot_haptic_teleop/haptic_proxy.h"

HapticProxy::HapticProxy(double r1, double r2, double r3, float k, const std::string& frame_id)
{
  double octree_resolution = 512.0;
  octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(octree_resolution));
  // Set initial position for Proxy and HIP
  hip_position = Eigen::Vector3f::Zero();
  proxy_position = Eigen::Vector3f::Zero();
  // Set normal init state
  normal = Eigen::Vector3f::Zero();
  valid_normal = false;
  // Set force init state
  force = Eigen::Vector3f::Zero();
  // Set initial state as Free
  proxy_state = ProxyState::FREE;
  // Set inital state for hip as Outside
  hip_state = HipState::OUTSIDE;
  // Update radius
  setRadius(r1, r2, r3);
  this->frame_id = frame_id;
  // Spring constant
  this->k = k;
}

void HapticProxy::setRadius(const double r1, const double r2, const double r3)
{
  this->r1 = r1;
  this->r2 = r2;
  this->r3 = r3;
  ROS_INFO_STREAM("Setting r1 = " << r1 << ", r2 = " << r2 << ", r3 = " << r3);
}

void HapticProxy::getRadious(double& r1, double& r2, double& r3) const
{
  r1 = this->r1;
  r2 = this->r2;
  r3 = this->r3;
}

std::string HapticProxy::getFrameId() const
{
  return frame_id;
}

double HapticProxy::getSpringConstant() const
{
  return k;
}

void HapticProxy::setSpringConstant(float k)
{
  this->k = k;
}

/**
 * Update point cloud
 * @param cloud Input point cloud, must be converted to Haptic proxy frame
 */
void HapticProxy::updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // Delete previous information and regenerate octree based on new point cloud
  octree->deleteTree();
  octree->setInputCloud(cloud);
  octree->addPointsFromInputCloud();
}

void HapticProxy::setProxyPositionOverride(const double x, const double y, const double z)
{
  proxy_position.x() = x;
  proxy_position.y() = y;
  proxy_position.z() = z;
}  

void HapticProxy::setHipPosition(const double x, const double y, const double z)
{
  // Set position for Proxy
  Eigen::Vector3f hip_position_t0 = hip_position;
  hip_position.x() = x;
  hip_position.y() = y;
  hip_position.z() = z;
  // Update Hip position
  Eigen::Vector3f v = hip_position - proxy_position;
  float delta = (hip_position - hip_position_t0).norm();
  ROS_DEBUG_STREAM("Delta " << delta);
  ROS_DEBUG_STREAM("v " << v.x() << ", " <<  v.y() << ", "  << v.z());
  // Update normal
  updateNormal();
  // Update force
  updateForce(v);
  // Avoid update if delta it's less than a mm
  if(fabs(delta)< 0.0005) return;
  // Check proxy_ state and movement
  if(proxy_state == ProxyState::PC_FREE || proxy_state == ProxyState::FREE)
  {
    Eigen::Vector3f vhat = v.normalized();
    ROS_DEBUG_STREAM("vhat " << vhat.x() << ", " << vhat.y() << ", " << vhat.z());
    Eigen::Vector3f movement = delta * vhat;
    ROS_DEBUG_STREAM("Movement " << movement.x() << ", " << movement.y() << ", " << movement.z());
    proxy_position += movement;
    return;
  }
  else if(proxy_state == ProxyState::ENTRENCHMENT)
  {
    if(valid_normal)
    {
      ROS_DEBUG_STREAM("normal " << normal.x() << ", " << normal.y() << ", " << normal.z());
      Eigen::Vector3f movement = delta * normal;
      ROS_DEBUG_STREAM("Movement " << movement.x() << ", " << movement.y() << ", " << movement.z());
      proxy_position += movement;
    }
    return;
  }
  else if(proxy_state == ProxyState::CONTACT)
  {
    if(valid_normal)
    {
      updateHipState();
      if (hip_state == HipState::OUTSIDE)
      {
        Eigen::Vector3f vhat = v.normalized();
        ROS_DEBUG_STREAM("vhat " << vhat.x() << ", " << vhat.y() << ", " << vhat.z());
        Eigen::Vector3f movement = delta * vhat;
        ROS_DEBUG_STREAM("Movement " << movement.x() << ", " << movement.y() << ", " << movement.z());
        proxy_position += movement;
        return;
      }
      else if (hip_state == HipState::INSIDE)
      {
        // Estimate v_plane (normal it's normalized, so n dot n = 1)
        Eigen::Vector3f vplane = v - v.dot(normal)*normal;
        ROS_DEBUG_STREAM("vplane " << vplane.x() << ", " << vplane.y() << ", " << vplane.z());
        Eigen::Vector3f movement = delta * vplane;
        ROS_DEBUG_STREAM("Movement " << movement.x() << ", " << movement.y() << ", " << movement.z());
        proxy_position += movement;
        return;

      }
    }
    return;
  }
}

bool HapticProxy::updateHipState()
{
  if (!valid_normal) return false;
  // Get plane from normal and centroid
  // From model ax + by + cz + d = 0 => d = -(ax + by + cz) = - normal dot p
  float d = - normal.dot(centroid);
  ROS_DEBUG_STREAM("d " << d);
  // From http://mathworld.wolfram.com/Point-PlaneDistance.html
  float D = normal.dot(hip_position) + d;
  ROS_DEBUG_STREAM("D " << D);
  // Normal and hip position are in the same side of the plane => Outside
  hip_state = D > 0.0f ? HipState::OUTSIDE : HipState::INSIDE;
  return true;
}

void HapticProxy::getProxyPosition(geometry_msgs::PoseStamped& pose) const
{
  pose.header.frame_id = frame_id;
  pose.pose.position.x = proxy_position.x();
  pose.pose.position.y = proxy_position.y();
  pose.pose.position.z = proxy_position.z();
  // Default orientation
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
}

void HapticProxy::getHipPosition(geometry_msgs::PoseStamped& pose) const
{
  pose.header.frame_id = frame_id;
  pose.pose.position.x = hip_position.x();
  pose.pose.position.y = hip_position.y();
  pose.pose.position.z = hip_position.z();
  // Default orientation
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
}

void HapticProxy::updateProxyPoints()
{
  // Clear index arrays
  point_idx_r1.clear();
  point_idx_r2.clear();
  point_idx_r3.clear();
  point_distance_r1.clear();
  point_distance_r2.clear();
  point_distance_r3.clear();
  // Get values from octree
  pcl::PointXYZ proxy_position_pcl;
  proxy_position_pcl.x = proxy_position.x();
  proxy_position_pcl.y = proxy_position.y();
  proxy_position_pcl.z = proxy_position.z();
  ROS_DEBUG_STREAM("Updated proxy_position " << proxy_position.x() << ", " <<  proxy_position.y() << ", "  << proxy_position.z());
  octree->radiusSearch(proxy_position_pcl, r1, point_idx_r1, point_distance_r1);
  ROS_DEBUG_STREAM("point_idx_r1.size() = "  << point_idx_r1.size());
  octree->radiusSearch(proxy_position_pcl, r2, point_idx_r2, point_distance_r2);
  ROS_DEBUG_STREAM("point_idx_r2.size() = "  << point_idx_r2.size());
  octree->radiusSearch(proxy_position_pcl, r3, point_idx_r3, point_distance_r3);
  ROS_DEBUG_STREAM("point_idx_r3.size() = "  << point_idx_r3.size());
}

void HapticProxy::updateProxyState()
{
  // Update proxy_ state based on the amount of points in each sphere

  // Check for free state, no points in bigger sphere
  if(point_idx_r3.size() == 0)
    proxy_state = ProxyState::PC_FREE;
  else if(point_idx_r2.size() == 0)
    proxy_state = ProxyState::FREE;
  else if(point_idx_r2.size() != 0 && point_idx_r1.size() == 0)
    proxy_state = ProxyState::CONTACT;
  else if(point_idx_r1.size() != 0)
    proxy_state = ProxyState::ENTRENCHMENT;
  ROS_DEBUG_STREAM("Current status: " << getProxyStateName());
}

void HapticProxy::updateForce(const Eigen::Vector3f &v)
{
  float vnorm = v.norm();
  ROS_DEBUG_STREAM("vnorm = " << vnorm);
  ROS_DEBUG_STREAM("Force criteria 0.5*(r1+r2) = " << 0.5*(r1+r2));
  // Check distance and proxy_ status
  if(vnorm < 0.5*(r1+r2) || proxy_state == ProxyState::PC_FREE || proxy_state == ProxyState::FREE)
  {
    ROS_DEBUG("Setting force zero due to hip it's close to proxy");
    force = Eigen::Vector3f::Zero();
    return;
  }
  // Force algorithm
  force = -v.normalized()*k*(vnorm-0.5*(r1+r2));
  float force_norm = force.norm();
  ROS_DEBUG_STREAM("Force: " << force_norm);
  // Check for small force
  if (force.norm()<0.1)
  {
    ROS_INFO("Setting force to zero due to force < 0.1");
    force = Eigen::Vector3f::Zero();
  }
}

void HapticProxy::getForce(Eigen::Vector3f& force) const
{
  force = this->force;
}

bool HapticProxy::updateNormal()
{
  if (point_idx_r3.size() <= 3)
  {
    ROS_DEBUG("No enough points to estimate normal");
    valid_normal = false;
    return valid_normal;
  }
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud =  octree->getInputCloud();
  Eigen::Vector3f vector_sum = Eigen::Vector3f::Zero();
  // Set to zero
  normal = Eigen::Vector3f::Zero();
  centroid = Eigen::Vector3f::Zero();
  for(std::size_t i=0; i < point_idx_r3.size(); ++i)
  {
    // Normal calculation
    Eigen::Vector3f pk = cloud->points[point_idx_r3[i]].getVector3fMap();
    Eigen::Vector3f diff = proxy_position - pk;
    vector_sum += diff.normalized();
    // Average point in the point cloud
    centroid += pk;
  }
  normal = 1.0/point_idx_r3.size()*vector_sum;
  centroid = 1.0/point_idx_r3.size()*centroid;
  this->normal = normal.normalized();
  this->centroid = centroid;
  valid_normal = true;
  return valid_normal;
}

bool HapticProxy::getNormal(Eigen::Vector3f &normal) const
{
  normal = this->normal;
  return valid_normal;
}

std::string HapticProxy::getProxyStateName() const
{
  std::string state_name;
  switch (proxy_state)
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

HapticProxy::HipState::Type HapticProxy::getHipState() const
{
  return hip_state;
}

std::string HapticProxy::getHipStateName() const
{
  std::string state_name;
  switch (hip_state)
  {
    case HipState::OUTSIDE:
      state_name = "OUTSIDE";
      break;
    case HipState::INSIDE:
      state_name = "INSIDE";
      break;
    case HipState::UNDEFINED:
      state_name = "UNDEFINED";
      break;
  }
  return state_name;
}

HapticProxy::ProxyState::Type HapticProxy::getProxyState() const
{
  return proxy_state;
}


