#include <ads1x1x.h>
#include <ads1115_tx2_i2c_interface.h>
#include <ros/ros.h>

//TODO hook up these addresses
//TODO hook up 2nd adc
static const uint8_t ADC_A = 0;
static uint8_t current = 255;

//TODO hook up 2nd adc
ADCInterface adc_a{ADC_A};

// Create an ADC object.
ADS1x1x_config_t my_adc;

// I2C stubs, implementations to be provided by the user.
uint8_t ADS1x1x_i2c_start_write(uint8_t i2c_address)
{
    //TODO hook up 2nd adc
    current = i2c_address;
    if (i2c_address == ADC_A)
        return (adc_a.isEnabled()) ? 0 : -1;
    return 0;
}

uint8_t ADS1x1x_i2c_write(uint8_t data)
{
    //TODO hook up 2nd adc
    if (current == ADC_A)
        return (adc_a.i2cWrite(data)) ? 0 : -1; 
    return 0;
}

uint8_t ADS1x1x_i2c_start_read(uint8_t i2c_address, uint16_t bytes_to_read)
{
    //TODO hook up 2nd adc
    current = i2c_address;
    if (current == ADC_A)
        return (adc_a.isEnabled()) ? 0 : -1;
    return 0;
}

uint8_t ADS1x1x_i2c_read(void)
{
    //TODO hook up 2nd adc
    if (current == ADC_A)
        return adc_a.i2cRead();
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
    //Initialise ADC object.
    if (ADS1x1x_init(&my_adc,ADS1115,ADS1x1x_I2C_ADDRESS_ADDR_TO_GND,MUX_SINGLE_0,PGA_4096)==0)
    {
        ROS_WARN("Could not initialise ADC structure.");
    }

    ros::spin();
    return 0;
}
