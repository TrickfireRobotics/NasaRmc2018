/**
 * Simple example to bring up a node and blink an led at address 0 on the i2c
 * pwm bus and demo the driver.
 * */

#include <ros/ros.h>
#include <ros/console.h>
#include <JHPWMPCA9685.h>

/**
 *  Convenience function for conceptualizing output.
 *  Takes in a number from -100 -> 100 specifying the desired output strength,
 *  where -100 = maximum speed reverse, 0 = neutral, and 100 is full speed
 *  ahead.
 *  Actual PWM outs for our cim
 *  340 = full reverse
 *  520 = stop
 *  700 = full ahead
 *  not sure our production cim's will be at all similar, so this is pretty
 *  temporary
 * */
int map(int x)
{
    //checks input
    if (x < -100 || x > 100)
        return 0;
    double magnitude = x * 180.0/100.0;
    long rounded = std::lround(magnitude);
    return 520 + static_cast<int>(rounded);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "blink");
    ros::NodeHandle n;
    ROS_INFO("Starting node");

    //min and max values for led, max = 4096
    int min = 0;
    int max = 666;


    std::unique_ptr<PCA9685> pca9685 (new PCA9685());

    int err = pca9685->openPCA9685();

    if (err < 0)
    {
        ROS_WARN("Error: %d", pca9685->error);
    } 
    else 
    {
        ROS_INFO("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress);
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        //max 999, 
        pca9685->setPWMFrequency(80) ;
        // 27 is the ESC key
        for (int j = -100; j <= 100; j +=10)
        {
            ROS_INFO("cycle %d", j);
            pca9685->setPWM(0,0,map(j));
            sleep(1);
        }
        pca9685->setPWM(0,0,map(0));
    }
    return 0;
}


