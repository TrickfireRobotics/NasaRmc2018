#ifndef ADS1115_TX2_I2C_INTERFACE_h
#define ADS1115_TX2_I2C_INTERFACE_h

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstdint>
#include <ads1x1x.h>
#include <ros/console.h>


/*I2C stubs, implementations in implementation file
  *do not explicitly need to be forward declared, but included in header for
  *maintainabilty */
class ADCInterface
{

    public:
        ADCInterface(uint8_t address);
        bool isEnabled();
        void writeWord(uint8_t reg, uint16_t data);
        uint16_t readWord(uint8_t reg);
        ADS1x1x_config_t config;
    private:
        uint8_t address;
        uint8_t i2cBus = 1 ;           // Default I2C bus for Jetson TK1
        int32_t error;
        int32_t fileDescriptor;
        bool enabled;
};

#endif
