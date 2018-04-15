#include <Adafruit_PWMServoDriver.h>
#include <Encoder.h>
#include <Wire.h>
#include <ros.h>
#include <tfr_msgs/ArduinoBReading.h>
#include <tfr_utilities/control_code.h>
#include <quadrature.h>
#include <tfr_msgs/PwmCommand.h>

ros::NodeHandle nh;

//encoder level constants
const double CPR = 4096; //pulse per revolution
const double GEARBOX_MPR = 3.33; 
const double TURNTABLE_RPR = 60;

//pin constants
const int GEARBOX_RIGHT_A = 2;
const int GEARBOX_RIGHT_B = 3;

enum class Address
{
    TREAD_LEFT = 0,
    TREAD_RIGHT = 1,
    ARM_TURNTABLE = 2,
    ARM_LOWER_LEFT = 3,
    ARM_LOWER_RIGHT = 4,
    ARM_UPPER = 5,
    ARM_SCOOP = 6,
    BIN_LEFT = 7,
    BIN_RIGHT = 8
};


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//encoders
VelocityQuadrature gearbox_right(CPR, GEARBOX_RIGHT_A, GEARBOX_RIGHT_B);
//TODO Quadrature turntable(CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::ArduinoBReading arduino_reading;

ros::Publisher arduino("arduino_b", &arduino_reading);
void motorOutput(const tfr_msgs::PwmCommand& command);
ros::Subscriber<tfr_msgs::PwmCommand> motor_subscriber("motor_output", &motorOutput );

int pwm_values[9] {};


void setup()
{
    nh.initNode();
    nh.advertise(arduino);
    nh.subscribe(motor_subscriber);
    pwm.begin();
    pwm.setPWMFreq(80);  // This is the maximum PWM frequency
    setAddress(Address::TREAD_LEFT, 0);
    setAddress(Address::TREAD_RIGHT, 0);
    setAddress(Address::ARM_TURNTABLE, 0);
    setAddress(Address::ARM_LOWER_LEFT, 0);
    setAddress(Address::ARM_LOWER_RIGHT, 0);
    setAddress(Address::ARM_UPPER, 0);
    setAddress(Address::ARM_SCOOP, 0);
    setAddress(Address::BIN_LEFT, 0);
}

void loop()
{
    arduino_reading.tread_right_vel = gearbox_right.getVelocity()*GEARBOX_MPR;
    //TODO    arduinoReading.arm_turntable_pos = turntable.getVelocity()/TURNTABLE_RPR;
    delay(8);
    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    delay(8);
    nh.spinOnce();
    delay(8);
    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    arduino.publish(&arduino_reading);
}

void motorOutput(const tfr_msgs::PwmCommand& command)
{
    if(command.enabled)
    {
        setAddress(Address::TREAD_LEFT, command.tread_left);
        setAddress(Address::TREAD_RIGHT, command.tread_right);
        setAddress(Address::ARM_TURNTABLE, command.arm_turntable);
        setAddress(Address::ARM_LOWER_LEFT, command.arm_lower_left);
        setAddress(Address::ARM_LOWER_RIGHT, command.arm_lower_right);
        setAddress(Address::ARM_UPPER, command.arm_upper);
        setAddress(Address::ARM_SCOOP, command.arm_scoop);
        setAddress(Address::BIN_LEFT, command.bin_right);
    }
    else
    {
        setAddress(Address::TREAD_LEFT, 0);
        setAddress(Address::TREAD_RIGHT, 0);
        setAddress(Address::ARM_TURNTABLE, 0);
        setAddress(Address::ARM_LOWER_LEFT, 0);
        setAddress(Address::ARM_LOWER_RIGHT, 0);
        setAddress(Address::ARM_UPPER, 0);
        setAddress(Address::ARM_SCOOP, 0);
        setAddress(Address::BIN_LEFT, 0);
    }

}


/*
 * sets a pwm output scaled between -1 and 1. Limits output change to 5% per cycle.
 */
void setAddress(const Address &addr, float val)
{
    //checks input
    if (val < -1 || val > 1)
        return;

    int address = static_cast<int>(addr);

    //translate from input value to pwm
    float magnitude = val * 180.0;
    uint16_t pwm_signal = 520 + round(magnitude);

    //scale the value to control change
    int sign = (pwm_signal-pwm_values[address] > 0) ? 1 : -1;
    if (abs(pwm_signal-pwm_values[address]) > 5)
        pwm_signal = pwm_values[address] + sign*5;
        
    pwm_values[address] = pwm_signal;
    pwm.setPWM(address, 0, pwm_signal);
}
