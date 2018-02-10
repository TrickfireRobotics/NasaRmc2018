/**
 * Simple example to bring up a node and blink an led at address 0 on the i2c
 * pwm bus and demo the driver.
 * */

#include <ros/ros.h>
#include <ros/console.h>
#include <JHPWMPCA9685.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blink");
    ros::NodeHandle n;
    ROS_INFO("Starting node");

    //min and max values for led, max = 4096
    int min = 400 ;
    int max = 4096 ;


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
        pca9685->setPWMFrequency(800) ;
        // 27 is the ESC key
        printf("Hit ESC key to exit\n");
        int i = 0;
        while(ros::ok())
        {
            ROS_INFO("cycle %d", i++);
            pca9685->setPWM(0,0,min);
            sleep(1);
            pca9685->setPWM(0,0,max);
            sleep(1);
        }
    }
    return 0;
}
