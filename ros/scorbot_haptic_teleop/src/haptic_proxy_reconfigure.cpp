#include "scorbot_haptic_teleop/haptic_proxy_reconfigure.h"

HapticProxyReconfig::HapticProxyReconfig(HapticProxyPtr proxy, ros::NodeHandle nh):
        proxy_(proxy),
        nh_("haptic_proxy")
{
  param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, nh_));
  /* Set dynamic reconfigure callback */
  param_reconfig_callback_ = boost::bind(&HapticProxyReconfig::dynamicReconfigCallback, this, _1, _2);
  param_reconfig_server_->setCallback(param_reconfig_callback_);
  dynamic_reconfig_initialized_ = true; /* We avoid first call of Dynamic reconfigure */
};

void HapticProxyReconfig::dynamicReconfigCallback(scorbot_haptic_teleop::HapticProxyParametersConfig &config, uint32_t level)
{
  if (!dynamic_reconfig_initialized_)
  {
    ROS_WARN("Haptic proxy dynamic reconfigure not initialized.");
    return;
  }
  proxy_->setRadius(config.r1,config.r2,config.r3);
  proxy_->setSpringConstant(config.k);

  // Set values back using a shared mutex with dynamic reconfig
  param_reconfig_mutex_.lock();
  param_reconfig_server_->updateConfig(config);
  param_reconfig_mutex_.unlock();
}

