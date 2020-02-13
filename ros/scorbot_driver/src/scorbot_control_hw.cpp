#include "scorbot_driver/scorbot_hardware_interface.h"
#include "scorbot_driver/scorbot_control_loop.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scorbot_control_hw");
  ros::NodeHandle nh;

  ros::NodeHandle nh_local("~");
  std::string ecat_iface;
  nh_local.param<std::string>("ethercat_iface", ecat_iface, "eth0");
  ROS_INFO_STREAM("Using " << ecat_iface << " as EtherCAT interface for Scorbot");

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<scorbot_driver::ScorbotHardwareInterface>
          scorbot_hw_interface = boost::make_shared<scorbot_driver::ScorbotHardwareInterface>(ecat_iface);
  scorbot_hw_interface->init();

  // Start the control loop
  scorbot_driver::ScorbotLoop control_loop(nh, scorbot_hw_interface);

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
