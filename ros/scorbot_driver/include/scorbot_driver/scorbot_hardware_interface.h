#ifndef SCORBOT_DRIVER_SCORBOT_HARDWARE_INTERFACE_H
#define SCORBOT_DRIVER_SCORBOT_HARDWARE_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

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

      // not implemented
      ScorbotHardwareInterface(ScorbotHardwareInterface const&);

      // not implemented
      ScorbotHardwareInterface& operator=(ScorbotHardwareInterface const&);

      // ROS hardware interface instances
      hardware_interface::JointStateInterface _jnt_state_interface;
      hardware_interface::PositionJointInterface _jnt_pos_interface;
      // For the gripper effort interface it's used
      hardware_interface::EffortJointInterface _jnt_eff_interface;

      // Memory space shared with the controller
      // It reads here the latest robot's state and put here the next desired values
      std::vector<std::string> _joint_names;
      std::vector<double> _prev_commands;
      std::vector<double> _joint_commands; // target joint angle
      std::vector<double> _joint_angles; // actual joint angle
      std::vector<double> _joint_velocities; // actual joint velocity
      std::vector<double> _joint_efforts; // compulsory but not used
      /* Gripper */
      bool _enable_gripper;
      double _gripper_command;
      double _gripper_angle;
      double _gripper_velocity;
      double _gripper_effort;

      // EtherCAT connection
      EthercatMaster _master;
      std::vector<ScorbotJointDriverPtr> _motors;
      ScorbotGripperDriverPtr _gripper;
      std::vector<std::string> _motor_names;
  };
}

#endif //SCORBOT_DRIVER_SCORBOT_HARDWARE_INTERFACE_H
