#ifndef SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H
#define SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H

#include "scorbot_driver/ethercat_driver.h"

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
                _joint_name(joint_name)
        {
          /* Init struct at zero */
          _set_point.controlRegA = 0U;
          _set_point.controlRegB = 0U;
          _set_point.currentRef = 0;
          _set_point.currentLim = 500;
          _set_point.pidCurrentKp = 0U;
          _set_point.pidCurrentKi = 0U;
          _set_point.pidCurrentKd = 0U;

          _joint_data.encPosition = 0;
          _joint_data.encSpeed = 0;
          _joint_data.current = 0;
          _joint_data.limits = 0U;
        };

        void setPosition(int16_t target)
        {
          /* TODO Apply limits */
          _set_point.currentRef = target;
        }

        int16_t getPosition()
        {
          return _joint_data.encPosition;
        }

        void update()
        {
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

    private:
        std::string _joint_name;
        setpoint_t _set_point;
        joint_data_t _joint_data;
    };

    typedef boost::shared_ptr<ScorbotJointDriver> ScorbotJointDriverPtr;
}

#endif //SCORBOT_DRIVER_SCORBOT_JOINT_DRIVER_H_H
