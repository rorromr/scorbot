#ifndef SCORBOT_DRIVER_ETHERCAT_DRIVER_H
#define SCORBOT_DRIVER_ETHERCAT_DRIVER_H

extern "C"
{
#include "ethercat.h"
}
#include <iomanip>

namespace scorbot_driver
{
    class EthercatDriver
    {
    public:
        EthercatDriver()
        {};

        virtual ~EthercatDriver()
        {};

        const std::string& getName() const
        {
          return _name;
        }

        virtual void update()=0;

        virtual bool configure(ec_slavet* mem_loc, uint32_t slave_nr)
        {
          _datap = mem_loc;
          std::stringstream stream;
          stream << "slave_" << std::hex << _datap->configadr;
          _name = stream.str();
          _slave_nr = slave_nr;
          return true;
        };

        virtual bool start()
        {
          return true;
        };

    public:
        ec_slavet* _datap;
        std::string _name;
        uint32_t _slave_nr;
    };

    typedef boost::shared_ptr<EthercatDriver> EthercatDriverPtr;
}


#endif //SCORBOT_DRIVER_ETHERCAT_DRIVER_H
