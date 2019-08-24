
#include "scorbot_driver/scorbot_hardware_interface.h"

namespace scorbot_driver
{
    ScorbotHardwareInterface::ScorbotHardwareInterface(const std::string& ifname):
            _master(ifname)
    {
      /* Fill motor names */
      _motor_names.clear();
      _motor_names.push_back("base");
      _motor_names.push_back("shoulder");
      _motor_names.push_back("elbow");
      _motor_names.push_back("pitch");
      _motor_names.push_back("roll");
      _enable_gripper = false;
    }

    ScorbotHardwareInterface::~ScorbotHardwareInterface()
    {
      /* TODO Hold current position */
    }

    void ScorbotHardwareInterface::update()
    {
      /* Update motors */
      for(std::size_t i = 0; i < _motors.size(); ++i)
      {
        _motors[i]->setPosition(_joint_commands[i]);
        _joint_angles[i] = _motors[i]->getPosition();
        _joint_efforts[i] = _motors[i]->getCurrent();
      }
      /* Update gripper */
      if (_enable_gripper)
      {
        _gripper->setCurrent(_gripper_command);
        _gripper_effort = _gripper->getCurrent();
        _gripper_angle =  _gripper->getPosition();
      }
      _master.update();
    }

    void ScorbotHardwareInterface::init()
    {
      /* Get number of motors */
      std::size_t motor_count = _motor_names.size();
      /* Resize vectors */
      _motors.resize(motor_count);
      _prev_commands.resize(motor_count, 0.0);
      _joint_commands.resize(motor_count, 0.0);
      _joint_angles.resize(motor_count, 0.0);
      _joint_velocities.resize(motor_count, 0.0);
      _joint_efforts.resize(motor_count, 0.0);

      for(std::size_t i = 0; i < _motors.size(); ++i)
      {
        /* Create joint drivers */
        _motors[i].reset(new ScorbotJointDriver(_motor_names[i]));
        /* Register joint driver on EtherCAT master */
        _master.registerDriver(boost::static_pointer_cast<EthercatDriver>(_motors[i]));
        /* Set ROS control memory address for JointStateInterface, using this ROS control
         * can read the joint information (joint angle, velocity and effort)
         */
        hardware_interface::JointStateHandle state_handle(
                _motors[i]->getJointName(),
                &_joint_angles[i],
                &_joint_velocities[i],
                &_joint_efforts[i]);
        _jnt_state_interface.registerHandle(state_handle);

        /* Set ROS control memory address for PositionJointInterface, using this ROS control
         * can set joint command (set point)
         */
        hardware_interface::JointHandle pos_handle(
                _jnt_state_interface.getHandle(_motors[i]->getJointName()),
                &_joint_commands[i]);
        _jnt_pos_interface.registerHandle(pos_handle);

        /* Hold current position */
        int16_t current_pos = _motors[i]->getPosition();
        _motors[i]->setPosition(0);//cHANGE!
      }
      /* Gripper */
      if (_enable_gripper)
      {
        /* Create joint drivers */
        _gripper.reset(new ScorbotGripperDriver());
        /* Register joint driver on EtherCAT master */
        _master.registerDriver(boost::static_pointer_cast<EthercatDriver>(_gripper));
        /* State interface */
        hardware_interface::JointStateHandle state_handle_right(
          "gripper_finger_right_joint",
          &_gripper_angle,
          &_gripper_velocity,
          &_gripper_effort);
        hardware_interface::JointStateHandle state_handle_left(
            "gripper_finger_left_joint",
            &_gripper_angle,
            &_gripper_velocity,
            &_gripper_effort);
        _jnt_state_interface.registerHandle(state_handle_right);
        _jnt_state_interface.registerHandle(state_handle_left);
        /* EffortJointInterface for the Gripper */
        hardware_interface::JointHandle eff_handle(
                  _jnt_state_interface.getHandle("gripper_finger_right_joint"),
                  &_gripper_command);
        _jnt_eff_interface.registerHandle(eff_handle);
      }
      // Init EtherCAT master
      _master.configure();
      _master.start();
      // Register the hardware interfaces on RobotHW
      registerInterface(&_jnt_state_interface);
      registerInterface(&_jnt_pos_interface);
      if(_enable_gripper)
      {
        registerInterface(&_jnt_eff_interface);
      }
    }
}
