#include <pwm_interface.h>


PWMInterface::PWMInterface()
{
    gpioExport(gpioPin);
    gpioSetDirection(gpioPin, outputPin);
    int err = pca9685->openPCA9685();
    if (err < 0)
    {
        ROS_WARN("Error: %d", pca9685->error);
    }    
    ROS_INFO("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress);
    pca9685->setAllPWM(0,0) ;
    pca9685->reset() ;
    pca9685->setPWMFrequency(80) ;
};

PWMInterface::~PWMInterface()
{
    gpioUnexport(gpioPin);
    gpioClose(gpioPin);
}

void PWMInterface::set(Address address, double value )
{
    if (value < -1 || value > 1)
    {
        ROS_WARN("Incorrect Motor Value: %f", value);
        return;
    }
    double pwm_1 = value, pwm_0 = pwm_values[static_cast<int>(address)];
    double sign = ((pwm_1 - pwm_0) > 0) ? 1 : -1;
    double magnitude = std::min(std::abs(pwm_1-pwm_0), 0.03);
    pwm_1  = pwm_0 + sign * magnitude;
    pwm_values[static_cast<int>(address)] = pwm_1; 
    pca9685->setPWM(static_cast<int>(address),0,map(pwm_1));
}


/**
 *  Convenience function for conceptualizing output.
 *  Takes in a number from -1 -> 1 specifying the desired output strength,
 *  where -1 = maximum speed reverse, 0 = neutral, and 1 is full speed
 *  ahead.
 *  Actual PWM outs for our cim
 *  340 = full reverse
 *  520 = stop
 *  700 = full ahead
 *  not sure our production cim's will be at all similar, so this is pretty
 *  temporary
 * */
int PWMInterface::map(double x)
{
    //checks input
    if (x < -1 || x > 1)
        return 0;
    double magnitude = x * 180.0;
    long rounded = std::lround(magnitude);
    return 520 + static_cast<int>(rounded);
}
