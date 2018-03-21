#include <ads1x1x.h>
#include <ads1115_tx2_i2c_interface.h>
#include <ros/ros.h>

//TODO hook up these addresses
//TODO hook up 2nd adc
static const uint8_t ADC_A = 0x48;
static uint8_t current = 255;

//TODO hook up 2nd adc
ADCInterface adc_a{ADC_A};

void ADS1x1x_i2c_write_word(uint8_t i2c_address, uint8_t reg, uint16_t value)
{
    //TODO hook up other adc here
    if (i2c_address == ADC_A)
        adc_a.writeWord(reg, value);
}

uint16_t ADS1x1x_i2c_read_word(uint8_t i2c_address, uint8_t reg)
{
    //TODO hook up other adc here
    if (i2c_address == ADC_A)
        return adc_a.readWord(reg);
    return 0;
}




int main(int argc, char**argv)
{
    ros::init(argc,argv,"adc_manager");
    ros::NodeHandle n;
    //Initialise ADC object.
    if
        (ADS1x1x_init(&adc_a.config,ADS1115,ADS1x1x_I2C_ADDRESS_ADDR_TO_GND,MUX_SINGLE_0,PGA_4096)==0)
    {
        ROS_WARN("Could not initialise ADC structure.");
    }
    ROS_INFO("adc initialized");
    while(ros::ok())
    {
        //Set input before starting conversion.
        ADS1x1x_start_conversion(&adc_a.config);
        // Default sample rate is 128 samples/s, i.e.one sample every 7.8 ms.
        ros::Duration(0.028).sleep();
        // Display result as voltage (remember, the PGA was set to 4.096 V
        float output = (float)ADS1x1x_read(&adc_a.config);
        ROS_INFO("raw: %f, converted: %f", output, output*6.144/32767.0);
        ros::spinOnce();
    }

    return 0;
}
