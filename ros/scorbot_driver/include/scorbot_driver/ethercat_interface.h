#ifndef SCORBOT_DRIVER_ETHERCAT_INTERFACE_H
#define SCORBOT_DRIVER_ETHERCAT_INTERFACE_H

// ROS
#include <ros/ros.h>

extern "C"
{
#include "ethercat.h"
}
#include "scorbot_driver/ethercat_driver.h"

namespace scorbot_driver
{
    class EthercatMaster
    {
    public:
        EthercatMaster(const std::string& ifname);
        ~EthercatMaster();

        bool registerDriver(EthercatDriverPtr driver);
        bool configure();
        bool start();
        void update();

        const std::string& getInterfaceName();

    private:
        uint8_t _IOmap[4096];
        std::string _ifname;
        uint32_t _expecterWKC;
        std::vector<EthercatDriverPtr> _drivers;
    };
}

#endif //SCORBOT_DRIVER_ETHERCAT_INTERFACE_H
