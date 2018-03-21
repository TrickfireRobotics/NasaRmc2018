#include <ads1115_tx2_i2c_interface.h>

ADCInterface::ADCInterface(uint8_t addr):
    address{addr},
    enabled{true}
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", i2cBus);
    fileDescriptor = open(fileNameBuffer, O_RDWR);
    if (fileDescriptor < 0) {
        // Could not open the file
        error = errno ;
        enabled = false;
    }
    if (ioctl(fileDescriptor, I2C_SLAVE, address) < 0) {
        // Could not open the device on the bus
        error = errno ;
        enabled = false;
    }
}

uint8_t ADCInterface::i2cWrite(uint8_t data)
{
    if (enabled)
    {
        int toReturn = i2c_smbus_write_byte(fileDescriptor, data);
        if (toReturn < 0) {
            printf("Write Byte error: %d",errno) ;
            error = errno ;
            toReturn = -1 ;
        }
        return toReturn ;
    }
    printf("Disabled: %d",errno) ;
    return -1;
}

uint8_t ADCInterface::i2cRead()
{
    if (enabled)
    {
        int toReturn = i2c_smbus_read_byte(fileDescriptor);
        if (toReturn < 0) {
            printf("Read Byte error: %d",errno) ;
            error = errno ;
            toReturn = -1 ;
        }
        return toReturn ;
    }
    printf("Disabled: %d",errno) ;
    return -1;
}

bool ADCInterface::isEnabled()
{
    return enabled;
}

