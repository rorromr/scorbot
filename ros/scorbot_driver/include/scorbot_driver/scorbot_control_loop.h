#ifndef SCORBOT_DRIVER_SCORBOT_CONTROL_LOOP_H
#define SCORBOT_DRIVER_SCORBOT_CONTROL_LOOP_H

// Standard C++
#include <time.h>

// Boost shared pointer
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// ROS control
#include <controller_manager/controller_manager.h>

// The hardware interface to dynamixels
#include "scorbot_driver/scorbot_hardware_interface.h"

namespace scorbot_driver {
    // Used to convert seconds elapsed to nanoseconds
    static const double BILLION = 1000000000.0;

    class ScorbotLoop {
    public:
        ScorbotLoop(ros::NodeHandle& nh, boost::shared_ptr<scorbot_driver::ScorbotHardwareInterface> hardware_interface);

        /** Timed method that reads current hardware's state, runs the controller
            code once and sends the new commands to the hardware.
            Note: we do not use the TimerEvent time difference because it
                does NOT guarantee that the time source is strictly
                linearly increasing.
        **/
        void update(const ros::TimerEvent& e);

    private:
        // Startup and shutdown of the internal node inside a roscpp program
        ros::NodeHandle _nh;

        // Settings
        ros::Duration _desired_update_freq;
        double _cycle_time_error_threshold;

        // Timing
        ros::Timer _non_realtime_loop;
        ros::Duration _elapsed_time;
        double _loop_hz;
        struct timespec _last_time;
        struct timespec _current_time;

        /** ROS Controller Manager and Runner
        This class advertises a ROS interface for loading, unloading, starting, and
        stopping ros_control-based controllers. It also serializes execution of all
        running controllers in \ref update.
        **/
        boost::shared_ptr<controller_manager::ControllerManager> _controller_manager;

        // Abstract Hardware Interface for your robot
        boost::shared_ptr<scorbot_driver::ScorbotHardwareInterface> _hardware_interface;
    };
}

#endif //SCORBOT_DRIVER_SCORBOT_CONTROL_LOOP_H
