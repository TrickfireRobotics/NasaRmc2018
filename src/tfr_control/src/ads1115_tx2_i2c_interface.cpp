#include <ads1115_tx2_i2c_interface.h>

ADCInterface::ADCInterface(uint8_t addr):
    config{},
    address{addr},
    enabled{true}
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", i2cBus);
    fileDescriptor = open(fileNameBuffer, O_RDWR);
    if (fileDescriptor < 0) {
        // Could not open the file
        ROS_WARN("Issue grabbing file descriptor");
        error = errno ;
        enabled = false;
    }
    if (ioctl(fileDescriptor, I2C_SLAVE, address) < 0) {
        // Could not open the device on the bus
        ROS_WARN("Issue opening file descriptor");
        error = errno ;
        enabled = false;
    }
    ROS_INFO("success fd %d", fileDescriptor);
}

void ADCInterface::writeWord(uint8_t reg, uint16_t data)
{
    if (enabled)
    {
        int toReturn = i2c_smbus_write_word_data(fileDescriptor, reg, data);
        ROS_INFO("direct write output: out->%x, reg->%x, data->%d, err->%d",
                toReturn,reg,  data, errno);
        if (toReturn < 0) {
            printf("Write Byte error: %d",errno) ;
            error = errno ;
        }
        else
            return;
    }
    printf("Disabled: %d",errno) ;
}

uint16_t ADCInterface::readWord(uint8_t reg)
{
    if (enabled)
    {
        uint16_t toReturn = i2c_smbus_read_word_data(fileDescriptor, reg);

        //int toReturn = i2c_smbus_read_word_data(fileDescriptor, 0x00);
        ROS_INFO("direct lib output: reg->%x, output->%d, err->%d",reg, toReturn, errno);
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

