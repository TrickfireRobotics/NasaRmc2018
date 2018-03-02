/**
 * Simple example to bring up a node and blink an led at address 0 on the i2c
 * pwm bus and demo the driver.
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

        PWMInterface();
        PWMInterface(const PWMInterface&) = delete;
        PWMInterface& operator=(const PWMInterface&) = delete;
        PWMInterface(PWMInterface&&) = delete;
        PWMInterface& operator=(PWMInterface&&) = delete;
        ~PWMInterface();

        void set(Address address, double value );

        void enablePWM(bool value);

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
        int map(double x);
        std::unique_ptr<PCA9685> pca9685{new PCA9685()};
};
