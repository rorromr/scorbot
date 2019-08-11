#ifndef HAPTIC_PROXY_H
#define HAPTIC_PROXY_H

#include <pcl/octree/octree.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class HapticProxy {

public:
    struct ProxyState {
        typedef enum
        {
            FREE,
            CONTACT,
            ENTRENCHMENT,
            PC_FREE
        } Type;
    };
    struct HipState {
        typedef enum
        {
            INSIDE,
            OUTSIDE,
            UNDEFINED
        } Type;
    };

    HapticProxy(double r1, double r2, double r3, float k, const std::string& frame_id);

    void setRadius(const double r1, const double r2, const double r3);

    void getRadious(double& r1, double& r2, double& r3) const;

    std::string getFrameId() const;

    double getSpringConstant() const;

    void setSpringConstant(float k);

    /**
     * Update point cloud
     * @param cloud Input point cloud, must be converted to Haptic proxy frame
     */
    void updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void setProxyPositionOverride(const double x, const double y, const double z);

    void setHipPosition(const double x, const double y, const double z);

    bool updateHipState();

    void getProxyPosition(geometry_msgs::PoseStamped& pose) const;

    void getHipPosition(geometry_msgs::PoseStamped& pose) const;

    void updateProxyPoints();

    void updateProxyState();

    void updateForce(const Eigen::Vector3f &v);

    void getForce(Eigen::Vector3f& force) const;

    bool updateNormal();

    bool getNormal(Eigen::Vector3f &normal) const;

    std::string getProxyStateName() const;

    HipState::Type getHipState() const;

    std::string getHipStateName() const;

    ProxyState::Type getProxyState() const;

protected:
    // Radio
    double r1, r2, r3;
    // Octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree;
    Eigen::Vector3f proxy_position;
    Eigen::Vector3f hip_position;
    Eigen::Vector3f normal;
    Eigen::Vector3f centroid;
    Eigen::Vector3f force;
    bool valid_normal;
    // Spring constant
    float k;
    // For store octree results
    std::vector<int> point_idx_r1;
    std::vector<int> point_idx_r2;
    std::vector<int> point_idx_r3;
    std::vector<float> point_distance_r1;
    std::vector<float> point_distance_r2;
    std::vector<float> point_distance_r3;

    std::string frame_id;

    ProxyState::Type proxy_state;
    HipState::Type hip_state;
};
typedef boost::shared_ptr<HapticProxy> HapticProxyPtr;

#endif //HAPTIC_PROXY_H
