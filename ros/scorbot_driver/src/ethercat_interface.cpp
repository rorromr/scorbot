
#include <string>
#include <iostream>
#include <ros/ros.h>

extern "C"
{
#include "ethercat.h"
}

namespace scorbot_driver
{
  class SoemMaster
  {
    public:
      SoemMaster(const std::string& ifname):
              _ifname(ifname)
      {
      }

      boolean configure() {
        int ret = ec_init((char *) _ifname.c_str());
        /* Initialise SOEM */
        if (ret > 0)
        {
          ROS_INFO_STREAM("EtherCAT on " << _ifname.c_str() << " interface succeeded.");

          /* Found slaves and config */
          if ( ec_config_init(FALSE) > 0 )
          {
            /* Number slaves */
            ROS_INFO_STREAM(ec_slavecount << " slaves found and configured.");
            /* Wait for all slaves to reach SAFE_OP state */
            ROS_INFO("Slaves mapped, request for pre-operational state (PRE_OP).\n");
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);
            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            /* Create slave custom drivers */
            for (std::size_t i = 1; i <= ec_slavecount; i++)
            {
              ;
            }
            /* Map slaves */
            ec_config_map(&_IOmap);
            ec_configdc();
            while (EcatError)
            {
              ROS_ERROR_STREAM(ec_elist2string());
            }
          }
          else
          {
            ROS_ERROR_STREAM("Configuration of slaves failed using " << _ifname.c_str() << " interface.");
            return false;
          }
        }
        else
        {
          ROS_ERROR_STREAM("Could not initialize EtherCAT master on " << _ifname.c_str() << " interface.");
          return false;
        }
      }

      boolean start();

      void update();



      const std::string& getInterfaceName()
      {
        return _ifname;
      }

    private:
      uint8_t _IOmap[4096];
      std::string _ifname;


  };
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "scorbot_driver");
  scorbot_driver::SoemMaster ecat_master("eth0");
  ecat_master.configure();
}
