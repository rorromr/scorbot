#ifndef HAPTIC_PROXY_RECONFIGURE_H
#define HAPTIC_PROXY_RECONFIGURE_H

#include <dynamic_reconfigure/server.h>
#include <boost/thread/mutex.hpp>

#include "scorbot_haptic_teleop/HapticProxyParametersConfig.h"
#include "scorbot_haptic_teleop/haptic_proxy.h"


class HapticProxyReconfig
{
public:
    HapticProxyReconfig(HapticProxyPtr proxy, ros::NodeHandle nh);

    void dynamicReconfigCallback(scorbot_haptic_teleop::HapticProxyParametersConfig &config, uint32_t level);

private:
    HapticProxyPtr proxy_;
    ros::NodeHandle nh_;
    // Dynamics reconfigure
    bool dynamic_reconfig_initialized_;
    typedef dynamic_reconfigure::Server<scorbot_haptic_teleop::HapticProxyParametersConfig> DynamicReconfigServer;
    boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
    DynamicReconfigServer::CallbackType param_reconfig_callback_;
    boost::recursive_mutex param_reconfig_mutex_;
};

#endif
