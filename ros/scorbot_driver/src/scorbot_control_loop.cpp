/* Based on Dave Coleman (https://github.com/davetcoleman/ros_control_boilerplate) */

#include "scorbot_driver/scorbot_control_loop.h"
#include <stdexcept>


namespace scorbot_driver {
    ScorbotLoop::ScorbotLoop(
            ros::NodeHandle& nh,
            boost::shared_ptr<scorbot_driver::ScorbotHardwareInterface> hardware_interface)
            : _nh(nh), _hardware_interface(hardware_interface)
    {
      // Create the controller manager
      _controller_manager.reset(new controller_manager::ControllerManager(_hardware_interface.get(), _nh));

      // Load rosparams
      int error = 0;
      ros::NodeHandle np("~");
      error += !np.getParam("loop_frequency", _loop_hz);
      error += !np.getParam("cycle_time_error_threshold", _cycle_time_error_threshold);
      if (error > 0) {
        char error_message[] = "could not retrieve one of the required parameters\n\tscorbot_hw/loop_hz or scorbot_hw/cycle_time_error_threshold";
        ROS_ERROR_STREAM(error_message);
        throw std::runtime_error(error_message);
      }

      // Get current time for use with first update
      clock_gettime(CLOCK_MONOTONIC, &_last_time);

      // Start timer that will periodically call DynamixelLoop::update
      ros::Duration _desired_update_freq = ros::Duration(1 / _loop_hz);
      _non_realtime_loop = _nh.createTimer(_desired_update_freq, &ScorbotLoop::update, this);
    }

    void ScorbotLoop::update(const ros::TimerEvent& e)
    {
      // Get change in time
      clock_gettime(CLOCK_MONOTONIC, &_current_time);
      _elapsed_time = ros::Duration(
              _current_time.tv_sec - _last_time.tv_sec + (_current_time.tv_nsec - _last_time.tv_nsec) / BILLION);
      _last_time = _current_time;

      // Check cycle time for excess delay
      const double cycle_time_error = (_elapsed_time - _desired_update_freq).toSec();
      if (cycle_time_error > _cycle_time_error_threshold) {
        ROS_WARN_STREAM(std::string("Cycle time exceeded error threshold by: ")
                                << cycle_time_error - _cycle_time_error_threshold << ", cycle time: " << _elapsed_time
                                << ", threshold: " << _cycle_time_error_threshold);
      }

      // Hardware update
      _hardware_interface->update();

      // Control
      // let the controller compute the new command (via the controller manager)
      _controller_manager->update(ros::Time::now(), _elapsed_time);

    }
}