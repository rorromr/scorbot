#ifndef SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H
#define SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H

#include <realtime_tools/realtime_buffer.h>

#include "scorbot_driver/ethercat_driver.h"
#include "scorbot_driver/ScorbotJointParametersConfig.h"

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <control_toolbox/ParametersConfig.h>
#include <boost/thread/mutex.hpp>

namespace scorbot_driver {
    typedef struct PACKED {
        uint16_t controlRegA;
        uint16_t controlRegB;
        int16_t currentRef;
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

    class ScorbotJointDriver : public EthercatDriver {
    public:
        ScorbotJointDriver(const std::string &joint_name);

        void setPosition(double target);

        double getPosition();

        double getCurrent();
        // RT method
        void update();

        const std::string &getJointName() const;

        void dynamicReconfigCallback(scorbot_driver::ScorbotJointParametersConfig &config, uint32_t level);

    private:
        static const double CURRENT_FACTOR;
        double ENC2RAD;
        double RAD2ENC;
        int32_t _encoder_tick_per_revolution;
        std::string _joint_name;
        setpoint_t _set_point;
        joint_data_t _joint_data;

        std::map<std::string, uint32_t> _level_mapping;
        // Current limit
        realtime_tools::RealtimeBuffer<int16_t> _current_limit_;
        // Calibration offset
        realtime_tools::RealtimeBuffer<int16_t> _calibration_offset_;
        // Dynamics reconfigure
        bool _dynamic_reconfig_initialized;
        typedef dynamic_reconfigure::Server<scorbot_driver::ScorbotJointParametersConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> _param_reconfig_server;
        DynamicReconfigServer::CallbackType _param_reconfig_callback;
        boost::recursive_mutex _param_reconfig_mutex;
    };

    typedef boost::shared_ptr<ScorbotJointDriver> ScorbotJointDriverPtr;

    class ScorbotGripperDriver : public EthercatDriver {
    public:
        ScorbotGripperDriver();
        void setCurrent(double target);
        double getPosition();
        double getCurrent();
        void update();
    private:
        static const double CURRENT_FACTOR;
        static const double ENCODER_FACTOR;
        static const double ENCODER_OFFSET;
        setpoint_t _set_point;
        joint_data_t _joint_data;
    };
    typedef boost::shared_ptr<ScorbotGripperDriver> ScorbotGripperDriverPtr;
}

#endif //SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H_H
