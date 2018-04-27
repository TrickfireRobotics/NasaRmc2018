#include <Adafruit_PWMServoDriver.h>
#include <Encoder.h>
#include <Wire.h>
#include <ros.h>
#include <tfr_msgs/ArduinoBReading.h>
#include <quadrature.h>
#include <std_msgs/Int32.h>
#include <tfr_msgs/PwmCommand.h>

ros::NodeHandle nh;

//encoder level constants
const double CPR = 4096; //pulse per revolution
const double GEARBOX_MPR = 3.33; 

//pin constants
const int GEARBOX_RIGHT_A = 2;
const int GEARBOX_RIGHT_B = 3;
const int OUTPUT_ENABLE = 4;
const int NEUTRAL = 470;

enum class Address : int16_t
{
    TREAD_LEFT = 0,
    TREAD_RIGHT = 1,
    ARM_LOWER = 2,
    ARM_UPPER = 3,
    ARM_SCOOP = 4,
    ARM_TURNTABLE = 5,
    BIN_LEFT = 6,
    BIN_RIGHT = 7
};


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//encoders
VelocityQuadrature gearbox_right(CPR, GEARBOX_RIGHT_A, GEARBOX_RIGHT_B);

//TODO Quadrature turntable(CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::ArduinoBReading arduino_reading;
ros::Publisher arduino("arduino_b", &arduino_reading);
void motorOutput(const tfr_msgs::PwmCommand& command);
ros::Subscriber<tfr_msgs::PwmCommand> motor_subscriber("/motor_output", &motorOutput );

uint16_t pwm_values[9] {};


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
    setAddress(Address::ARM_LOWER, 0);
    setAddress(Address::ARM_UPPER, 0);
    setAddress(Address::ARM_SCOOP, 0);
    setAddress(Address::BIN_LEFT, 0);
    pinMode(OUTPUT_ENABLE, OUTPUT);
    for (auto& val : pwm_values)
        val = NEUTRAL;
}

void loop()
{
    arduino_reading.tread_right_vel = -gearbox_right.getVelocity()/GEARBOX_MPR;
    delay(8);
    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    delay(8);
    arduino.publish(&arduino_reading);
    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
}

void motorOutput(const tfr_msgs::PwmCommand& command)
{

    if(command.enabled)
    {
      	digitalWrite(OUTPUT_ENABLE, LOW);
        setAddress(Address::TREAD_LEFT, command.tread_left);
        setAddress(Address::TREAD_RIGHT, command.tread_right);
        setAddress(Address::ARM_TURNTABLE, command.arm_turntable);
        setAddress(Address::ARM_LOWER, command.arm_lower);
        setAddress(Address::ARM_UPPER, command.arm_upper);
        setAddress(Address::ARM_SCOOP, command.arm_scoop);
        setAddress(Address::BIN_LEFT, command.bin_right);
    }
    else
    {
      	digitalWrite(OUTPUT_ENABLE, HIGH);
        setAddress(Address::TREAD_LEFT, 0);
        setAddress(Address::TREAD_RIGHT, 0);
        setAddress(Address::ARM_TURNTABLE, 0);
        setAddress(Address::ARM_LOWER, 0);
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

    int16_t address = static_cast<int16_t>(addr);

    //translate from input value to pwm
    float magnitude = val * 170.0;

    //round the value
    int16_t rounded{};
    if (magnitude >= 0)
        rounded = static_cast<int16_t>(magnitude + 0.5);
    else
        rounded = static_cast<int16_t>(magnitude - 0.5);

    uint16_t pwm_signal = NEUTRAL + rounded;

    //scale the value to control change
    int16_t delta = pwm_signal-pwm_values[address];

    int16_t sign = (delta >= 0) ? 1 : -1;
    if (((delta >= 0) ? delta : -delta) > 15)
        pwm_signal = pwm_values[address] + (sign*15);
    
    pwm_values[address] = pwm_signal;
    pwm.setPWM(address, 0, pwm_signal);
}
