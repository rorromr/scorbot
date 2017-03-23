#ifndef SCORBOT_DRIVER_SCORBOT_HARDWARE_INTERFACE_H
#define SCORBOT_DRIVER_SCORBOT_HARDWARE_INTERFACE_H

// ROS
#include <ros/ros.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Custom EtherCAT master
#include "scorbot_driver/ethercat_interface.h"
#include "scorbot_driver/scorbot_joint_driver.h"

namespace scorbot_driver
{
  class ScorbotHardwareInterface : public hardware_interface::RobotHW
  {
    public:

      ScorbotHardwareInterface(const std::string& ifname);
      ~ScorbotHardwareInterface();

      void init();
      void update();

    private:
      static const double ENC2RAD = 2.0*M_PI/(3.0*160.0*96.0);
      static const double RAD2ENC = 1.0/(2.0*M_PI/(3.0*160.0*96.0));
      // not implemented
      ScorbotHardwareInterface(ScorbotHardwareInterface const&);

      // not implemented
      ScorbotHardwareInterface& operator=(ScorbotHardwareInterface const&);

      // ROS hardware interface instances
      hardware_interface::JointStateInterface _jnt_state_interface;
      hardware_interface::PositionJointInterface _jnt_pos_interface;

      // Memory space shared with the controller
      // It reads here the latest robot's state and put here the next desired values
      std::vector<std::string> _joint_names;
      std::vector<double> _prev_commands;
      std::vector<double> _joint_commands; // target joint angle
      std::vector<double> _joint_angles; // actual joint angle
      std::vector<double> _joint_velocities; // actual joint velocity
      std::vector<double> _joint_efforts; // compulsory but not used

      // EtherCAT connection
      EthercatMaster _master;
      std::vector<ScorbotJointDriverPtr> _motors;
      std::vector<std::string> _motor_names;
  };
}

#endif //SCORBOT_DRIVER_SCORBOT_HARDWARE_INTERFACE_H
