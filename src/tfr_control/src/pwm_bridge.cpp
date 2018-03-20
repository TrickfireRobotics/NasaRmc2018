/**
 * interface for the PWM board to be used by control library.
 * */

#include <ros/ros.h>
#include <ros/console.h>
#include <JHPWMPCA9685.h>
extern "C"
{
    #include <gpio.h>
}

class PWMInterface
{
    public:
        enum class Address
        {
            LEFT_TREAD = 0
        };

        PWMInterface()
        {
            gpioExport(gpio389);
            gpioSetDirection(gpio389, outputPin);
            enablePWM(false);
            ROS_INFO("here");
            int err = pca9685->openPCA9685();
            ROS_INFO("here %d", err);
            if (err < 0)
            {
                ROS_WARN("Error: %d", pca9685->error);
            }    
            ROS_INFO("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress);
            pca9685->setAllPWM(0,0) ;
            pca9685->reset() ;
            pca9685->setPWMFrequency(80) ;
        };

        ~PWMInterface()
        {
            gpioUnexport(gpio389);
            gpioClose(gpio389);
        }

        void set(Address address, double value )
        {
            if (value < -1 || value > 1)
            {
                ROS_WARN("Incorrect Motor Value: %f", value);
                return;
            }
            pca9685->setPWM(static_cast<int>(address),0,map(value));
        }

        void enablePWM(bool value)
        {
            gpioSetValue(gpio389, (value)? off : on);
        }

    private:
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
        int map(double x)
        {
            //checks input
            if (x < -100 || x > 100)
                return 0;
            double magnitude = x * 180.0;
            long rounded = std::lround(magnitude);
            return 520 + static_cast<int>(rounded);
        }

        std::unique_ptr<PCA9685> pca9685{new PCA9685()};
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "blink");
    ros::NodeHandle n;
    ROS_INFO("Starting node");

    PWMInterface interface{};
    interface.enablePWM(true);

    bool sign = false;
    for (double j = 0.055; true ; j += ((sign)?1:-1)*0.001)
    {
        ROS_INFO("cycle %f", j);
        interface.set(PWMInterface::Address::LEFT_TREAD, 0.055);
        sleep(0.25);
        if (j < 0.055)
            sign = true;
        if(j > 0.08)
            sign = false;
    }
    interface.set(PWMInterface::Address::LEFT_TREAD, 0);
    return 0;
}

