#ifndef HAPTIC_PROXY_MARKER_H
#define HAPTIC_PROXY_MARKER_H

#include "haptic_proxy.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

static const std::string HAPTIC_PROXY_MARKER_TOPIC = "/proxy_marker";

class HapticProxyMarker
{
protected:
    // Sphere markers
    visualization_msgs::Marker r1_marker_, r2_marker_, r3_marker_;
    // Arrow markers
    visualization_msgs::Marker normal_marker_, force_marker_;
    // Text marker
    visualization_msgs::Marker state_marker_;
    HapticProxyPtr proxy_;
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
public:
    HapticProxyMarker(HapticProxyPtr proxy, std::string marker_topic = HAPTIC_PROXY_MARKER_TOPIC,
                                         ros::NodeHandle nh = ros::NodeHandle("~"));

    void updateProxyMarker();
};

#endif //HAPTIC_PROXY_MARKER_H
