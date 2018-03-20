#include <ads1x1x.h>
#include <ads1115_tx2_i2c_interface.h>
#include <ros/ros.h>

//TODO hook up these addresses
static const uint8_t ADC_A = 0;
static uint8_t current = 255;

ADCInterface adc_a{ADC_A};

// I2C stubs, implementations to be provided by the user.
uint8_t ADS1x1x_i2c_start_write(uint8_t i2c_address)
{
    current = i2c_address;
    if (i2c_address == ADC_A)
        return (adc_a.isEnabled()) ? 0 : -1;
    return 0;
}

uint8_t ADS1x1x_i2c_write(uint8_t data)
{
    if (current == ADC_A)
        return (adc_a.i2cWrite(data)) ? 0 : -1; 
    return 0;
}

uint8_t ADS1x1x_i2c_start_read(uint8_t i2c_address, uint16_t bytes_to_read)
{
    return 0;
}
uint8_t ADS1x1x_i2c_read(void)
{
    return 0;
}
uint8_t ADS1x1x_i2c_stop(void)
{
    return 0;
}

int main(int argc, char**argv)
{
    ros::init(argc,argv,"adc_manager");
    ros::NodeHandle n;
    ros::spin();
    return 0;
}
