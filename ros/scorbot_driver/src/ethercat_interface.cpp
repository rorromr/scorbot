
#include <string>
#include <iostream>
#include <ros/ros.h>

#include "scorbot_driver/ethercat_interface.h"

namespace scorbot_driver
{
    EthercatMaster::EthercatMaster(const std::string& ifname):
          _ifname(ifname),
          _expecterWKC(0UL){}

    EthercatMaster::~EthercatMaster()
    {
      /* Request INIT state for all slaves */
      ec_slave[0].state = EC_STATE_INIT;
      ec_writestate(0);
      /* Number of attempts to reach INIT state */
      std::size_t init_attempts = 40;
      /* Wait for all slaves to reach OP state */
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_INIT, 50000);
      }
      while (init_attempts-- && (ec_slave[0].state != EC_STATE_INIT));
      if (ec_slave[0].state == EC_STATE_INIT )
      {
        ROS_INFO("Init state (INIT) reached for all slaves.");
      }
      else
      {
        ROS_WARN("Not all slaves reached init state.");
      }
      ec_close();
      ROS_INFO("EtherCAT closed.");
    }

    bool EthercatMaster::registerDriver(EthercatDriverPtr driver)
    {
      /* Add driver */
      _drivers.push_back(driver);
      ROS_INFO_STREAM("Register driver " << driver->getName().c_str());
    }

    bool EthercatMaster::configure() {
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
          ROS_INFO("Slaves mapped, request for pre-operational state (PRE_OP).");
          ec_slave[0].state = EC_STATE_PRE_OP;
          ec_writestate(0);
          ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
          /* Create slave custom drivers */
          std::size_t driver_size = _drivers.size();
          if (driver_size > ec_slavecount)
          {
            ROS_ERROR("Number of drivers exceeded the number of slaves.");
            return false;
          }
          if (driver_size < ec_slavecount)
          {
            ROS_WARN("Number of drivers is less than the number of slaves.");
          }
          for (std::size_t driver_idx = 0, slave_idx = 1;
               driver_idx < driver_size && slave_idx <= ec_slavecount;
               ++driver_idx, ++slave_idx)
          {
            _drivers[driver_idx]->configure(&ec_slave[slave_idx], slave_idx);
          }
          /* Map slaves */
          ec_config_map(&_IOmap);
          ec_configdc();
          while (EcatError)
          {
            ROS_ERROR_STREAM(ec_elist2string());
          }
          return true;
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

    bool EthercatMaster::start()
    {
      /* Request SAFE_OP */
      ROS_INFO("Request safe-operational (SAFE_OP) state for all slaves.");
      ec_slave[0].state = EC_STATE_SAFE_OP;
      ec_writestate(0);
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
      if (ec_slave[0].state == EC_STATE_SAFE_OP)
      {
        ROS_INFO("Safe-operational (SAFE_OP) state reached for all slaves.");
        while (EcatError)
        {
          ROS_ERROR_STREAM(ec_elist2string());
        }
      }
      else
      {
        /* Get slaves that did not reach SAFE_OP */
        ec_readstate();
        for (std::size_t i = 0; i <= ec_slavecount; i++)
        {
          if (ec_slave[i].state != EC_STATE_SAFE_OP)
          {
            ROS_ERROR_STREAM("Slave " << i << ": State=" << std::hex << ec_slave[i].state
                                      << " StatusCode="  << ec_slave[i].ALstatuscode << " : "
                                      << ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
        /* Not return false until try to reach OP */
      }
      /* Request OP */
      ROS_INFO("Request operational state (OP) for all slaves.");
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* Send one valid process data to make outputs in slaves happy */
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* Request OP state for all slaves */
      ec_writestate(0);
      while (EcatError)
      {
        ROS_ERROR_STREAM(ec_elist2string());
      }
      int attempts = 40;
      /* Wait for all slaves to reach OP state */
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      }
      while (attempts-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
      if (ec_slave[0].state == EC_STATE_OPERATIONAL )
      {
        ROS_INFO("Operational state (OP) reached for all slaves.");
        _expecterWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        ROS_INFO_STREAM("Calculated workcounter " << _expecterWKC);
      }
      else
      {
        /* Get slaves that did not reach OP */
        ec_readstate();
        for (std::size_t i = 0; i <= ec_slavecount; i++)
        {
          if (ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
            ROS_ERROR_STREAM("Slave " << i << ": State=" << std::hex << ec_slave[i].state
                                      << " StatusCode="  << ec_slave[i].ALstatuscode << " : "
                                      << ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
        return false;
      }
      return true;
    }

    void EthercatMaster::update()
    {
      bool success = true;


      if (ec_receive_processdata(EC_TIMEOUTRET)  < _expecterWKC)
      {
        success = false;
        ROS_WARN("Receiving data failed");
      }

      if (success)
      {
      	/* Update drivers */
        for(std::size_t i = 0; i < _drivers.size(); ++i)
        {
          _drivers[i]->update();
        }
      }

      if (ec_send_processdata() == 0)
      {
        success = false;
        ROS_WARN("Sending process data failed");
      }
      while (EcatError)
      {
        ROS_ERROR_STREAM(ec_elist2string());
      }


    }

    const std::string& EthercatMaster::getInterfaceName()
    {
      return _ifname;
    }
}


