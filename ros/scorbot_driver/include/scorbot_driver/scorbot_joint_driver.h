#ifndef SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H
#define SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H

#include <realtime_tools/realtime_buffer.h>

#include "scorbot_driver/ethercat_driver.h"
#include "scorbot_driver/MotorConfig.h"

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <control_toolbox/ParametersConfig.h>
#include <boost/thread/mutex.hpp>

namespace scorbot_driver
{
    typedef struct PACKED {
        uint16_t controlRegA;
        uint16_t controlRegB;
        int16_t  currentRef;
        int16_t currentLim;
        uint16_t pidCurrentKp;
        uint16_t pidCurrentKi;
        uint16_t pidCurrentKd;
	      float testFloat;
    } setpoint_t;

    typedef struct PACKED {
        int16_t encPosition;
        int16_t encSpeed;
        int16_t current;
        uint16_t limits;
    } joint_data_t;

    class ScorbotJointDriver : public EthercatDriver
    {
    public:
        ScorbotJointDriver(const std::string& joint_name):
                EthercatDriver(),
                _joint_name(joint_name),
                _dynamic_reconfig_initialized(false)

        {

          _name = joint_name + "_controller";
          /* Init struct at zero */
          _set_point.controlRegA = 0U;
          _set_point.controlRegB = 0U;
          _set_point.currentRef = 0;
          _set_point.currentLim = std::floor(MotorConfig::__getDefault__().current_limit*CURRENT_FACTOR);
          _set_point.pidCurrentKp = 0U;
          _set_point.pidCurrentKi = 0U;
          _set_point.pidCurrentKd = 0U;
          _set_point.testFloat = 0.0f;

          _joint_data.encPosition = 0;
          _joint_data.encSpeed = 0;
          _joint_data.current = 0;
          _joint_data.limits = 0U;

          // Create node handle for dynamic reconfigure
          ros::NodeHandle nh(_joint_name);
          ROS_DEBUG_STREAM_NAMED(_joint_name,"Initializing dynamic reconfigure in namespace " << nh.getNamespace());

          // Start dynamic reconfigure server
          _param_reconfig_server.reset(new DynamicReconfigServer(_param_reconfig_mutex, nh));
          _dynamic_reconfig_initialized = true;

          // Set callback
          _param_reconfig_callback = boost::bind(&ScorbotJointDriver::dynamicReconfigCallback, this, _1, _2);
          _param_reconfig_server->setCallback(_param_reconfig_callback);
        };

        void setPosition(float target)
        {
          /* TODO Apply limits */
          _set_point.currentRef = (int16_t) target;
        }

        int16_t getPosition()
        {
          return _joint_data.encPosition;
        }

        int16_t getCurrent()
        {
          return _joint_data.current;
        }

        // RT method
        void update()
        {
          // Get from RT buffers
          _set_point.currentLim = *_current_limit_.readFromRT();
          /* Get joint data */
          _joint_data = *((joint_data_t*)(_datap->inputs));
          /* Apply set point */
          setpoint_t* data_out = ((setpoint_t*) (_datap->outputs));
          *data_out = _set_point;
        }

        const std::string& getJointName() const
        {
          return _joint_name;
        }

        void dynamicReconfigCallback(scorbot_driver::MotorConfig &config, uint32_t level)
        {
          ROS_DEBUG_STREAM_NAMED(_joint_name,"Dynamics reconfigure callback recieved.");

          if(!_dynamic_reconfig_initialized)
              return;

          // Update values using RT buffer
          int16_t raw_value = std::floor(config.current_limit*CURRENT_FACTOR);
          ROS_INFO_STREAM_NAMED(_joint_name,"Setting current_limit for motor " << _joint_name << " at " << config.current_limit << " A (" << raw_value << " raw value)");
          _current_limit_.writeFromNonRT(raw_value);

          // Set values back using a shared mutex with dynamic reconfig
          _param_reconfig_mutex.lock();
          _param_reconfig_server->updateConfig(config);
          _param_reconfig_mutex.unlock();
        }

    private:
        static const double CURRENT_FACTOR = 160.0;
        std::string _joint_name;
        setpoint_t _set_point;
        joint_data_t _joint_data;
        // Current limit
        realtime_tools::RealtimeBuffer<int16_t> _current_limit_;
        // Dynamics reconfigure
        bool _dynamic_reconfig_initialized;
        typedef dynamic_reconfigure::Server<scorbot_driver::MotorConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> _param_reconfig_server;
        DynamicReconfigServer::CallbackType _param_reconfig_callback;
        boost::recursive_mutex _param_reconfig_mutex;
    };

    typedef boost::shared_ptr<ScorbotJointDriver> ScorbotJointDriverPtr;
}

#endif //SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H_H
