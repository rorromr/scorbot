#include "scorbot_driver/scorbot_joint_driver.h"

namespace scorbot_driver {

  const double ScorbotJointDriver::CURRENT_FACTOR = 160.0;

  ScorbotJointDriver::ScorbotJointDriver(const std::string &joint_name) :
        EthercatDriver(),
        _joint_name(joint_name),
        _dynamic_reconfig_initialized(false)
  {
    _name = joint_name + "_controller";
    /* Init struct at zero */
    _set_point.controlRegA = 0U;
    _set_point.controlRegB = 0U;
    _set_point.currentRef = 0;
    _set_point.currentLim = (int16_t) std::floor(
            ScorbotJointParametersConfig::__getDefault__().current_limit * CURRENT_FACTOR);
    _set_point.pidCurrentKp = 0U;
    _set_point.pidCurrentKi = 0U;
    _set_point.pidCurrentKd = 0U;
    _set_point.testFloat = 0.0f;

    _joint_data.encPosition = 0;
    _joint_data.encSpeed = 0;
    _joint_data.current = 0;
    _joint_data.limits = 0U;

    // Dynamic reconfigure level mapping
    const std::vector<ScorbotJointParametersConfig::AbstractParamDescriptionConstPtr> params = ScorbotJointParametersConfig::__getParamDescriptions__();
    for (std::vector<ScorbotJointParametersConfig::AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin();
         _i != params.end(); ++_i)
    {
      std::string name = (*_i)->name;
      uint32_t level = (*_i)->level;
      _level_mapping.insert(std::pair<std::string, uint32_t>(name, level));
    }
    // Init RT buffers
    _current_limit_.initRT((int16_t) std::floor(ScorbotJointParametersConfig::__getDefault__().current_limit * CURRENT_FACTOR));
    _calibration_offset_.initRT((int16_t) std::floor(ScorbotJointParametersConfig::__getDefault__().calibration_offset * RAD2ENC));
    // Create node handle for dynamic reconfigure
    ros::NodeHandle nh(_name);
    ROS_INFO_STREAM_NAMED(_name, "Initializing dynamic reconfigure in namespace " << nh.getNamespace());

    // Start dynamic reconfigure server
    _param_reconfig_server.reset(new DynamicReconfigServer(_param_reconfig_mutex, nh));
    /* Set dynamic reconfigure callback */
    _param_reconfig_callback = boost::bind(&ScorbotJointDriver::dynamicReconfigCallback, this, _1, _2);
    _param_reconfig_server->setCallback(_param_reconfig_callback);
    _dynamic_reconfig_initialized = true; /* We avoid first call of Dynamic reconfigure */
    /* Initial values for parameters with dynamic reconfigure */
    uint32_t initial_config_level = 0U;
    scorbot_driver::ScorbotJointParametersConfig initial_config;
    std::map<std::string,uint32_t >::iterator it;
    nh.param<double>("current_limit", initial_config.current_limit, ScorbotJointParametersConfig::__getDefault__().current_limit);
    initial_config_level |= (it = _level_mapping.find("current_limit")) != _level_mapping.end() ? it->second : 0;
    dynamicReconfigCallback(initial_config, initial_config_level); /* Call dynamic reconfigure to setup current limit */

    /* Get encoder configuration */
    int32_t  scorbot_default_encoder = 3*160*96; /* Default Scorbot value */
    nh.param<int32_t >("encoder_ticks_per_revolution", _encoder_tick_per_revolution, scorbot_default_encoder);
    ROS_INFO_STREAM("Joint "<< _joint_name << " has " << _encoder_tick_per_revolution << " ticks per revolution");
    ENC2RAD = 2.0*M_PI/(1.0*_encoder_tick_per_revolution);
    RAD2ENC = 1.0/(2.0*M_PI/(1.0*_encoder_tick_per_revolution));
  };

  void ScorbotJointDriver::setPosition(double target) {
    _set_point.currentRef = (int16_t) std::floor(target * RAD2ENC);
  }

  double ScorbotJointDriver::getPosition() {
    return _joint_data.encPosition*ENC2RAD;
  }

  double ScorbotJointDriver::getCurrent() {
    return _joint_data.current*1.0/CURRENT_FACTOR;
  }

  // RT method
  void ScorbotJointDriver::update() {
    // Get dynamic reconfigure from RT buffers
    _set_point.currentLim = *_current_limit_.readFromRT();
    _set_point.currentRef += *_calibration_offset_.readFromRT();
    /* Get joint data */
    _joint_data = *((joint_data_t *) (_datap->inputs));
    // Compensate offset
    _joint_data.encPosition -= *_calibration_offset_.readFromRT();
    /* Apply set point */
    setpoint_t *data_out = ((setpoint_t *) (_datap->outputs));
    *data_out = _set_point;
  }

  const std::string& ScorbotJointDriver::getJointName() const {
      return _joint_name;
  }

  void ScorbotJointDriver::dynamicReconfigCallback(scorbot_driver::ScorbotJointParametersConfig &config, uint32_t level) {
    ROS_DEBUG_STREAM("Dynamic reconfigure callback recieved with level " << level);

    if (!_dynamic_reconfig_initialized)
      return;
    // Update values using Dynamic reconfigure
    std::map<std::string,uint32_t >::iterator it;
    // Check current limit update
    it = _level_mapping.find("current_limit");
    if (it != _level_mapping.end() && it->second & level){
      int16_t raw_value = std::floor(config.current_limit * CURRENT_FACTOR);
      // Update values using RT buffer
      ROS_INFO_STREAM_NAMED(_joint_name,
                            "Setting current limit for motor " << _joint_name << " at " << config.current_limit
                                                               << " A (" << raw_value << " raw value)");
      _current_limit_.writeFromNonRT(raw_value);
    }
    // Check calibration offset
    it = _level_mapping.find("calibration_offset");
    if (it != _level_mapping.end() && it->second & level){
      int16_t raw_value = (int16_t) std::floor(config.calibration_offset * RAD2ENC);
      // Update values using RT buffer
      ROS_INFO_STREAM_NAMED(_joint_name,
                            "Setting calibration offset for motor " << _joint_name << " at " << config.calibration_offset
                                                               << " rad (" << raw_value << " raw value)");
      _calibration_offset_.writeFromNonRT(raw_value);
    }


    // Set values back using a shared mutex with dynamic reconfig
    _param_reconfig_mutex.lock();
    _param_reconfig_server->updateConfig(config);
    _param_reconfig_mutex.unlock();
  }

    const double ScorbotGripperDriver::CURRENT_FACTOR = 160.0;
    const double ScorbotGripperDriver::ENCODER_FACTOR = 0.018/1400;
    const double ScorbotGripperDriver::ENCODER_OFFSET = 1400;
    ScorbotGripperDriver::ScorbotGripperDriver() :
        EthercatDriver()
    {
      /* Init struct at zero */
      _set_point.controlRegA = 0U;
      _set_point.controlRegB = 0U;
      _set_point.currentRef = 0;
      _set_point.currentLim = (int16_t) std::floor(1.875 * ScorbotGripperDriver::CURRENT_FACTOR);
      _set_point.pidCurrentKp = 0U;
      _set_point.pidCurrentKi = 0U;
      _set_point.pidCurrentKd = 0U;
      _set_point.testFloat = 0.0f;

      _joint_data.encPosition = 0;
      _joint_data.encSpeed = 0;
      _joint_data.current = 0;
      _joint_data.limits = 0U;
    };

    void ScorbotGripperDriver::setCurrent(double target) {
      _set_point.currentRef = (int16_t) std::floor(target * ScorbotGripperDriver::CURRENT_FACTOR);
    }

    double ScorbotGripperDriver::getPosition() {
      return (-_joint_data.encPosition - ScorbotGripperDriver::ENCODER_OFFSET)*ScorbotGripperDriver::ENCODER_FACTOR;
    }

    double ScorbotGripperDriver::getCurrent() {
      return _joint_data.current*1.0/ScorbotGripperDriver::CURRENT_FACTOR;
    }

    // RT method
    void ScorbotGripperDriver::update() {
      /* Get gripper data */
      _joint_data = *((joint_data_t *) (_datap->inputs));
      /* Apply set point */
      setpoint_t *data_out = ((setpoint_t *) (_datap->outputs));
      *data_out = _set_point;
    }
}