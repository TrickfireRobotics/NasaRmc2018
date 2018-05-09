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
const double GEARBOX_MPR = 2*3.1415*0.15; 
const float MAX_DRIVEBASE_DELTA = 8.0;
const float MAX_ARM_DELTA = 15.0;

const float FULL_DELTA = 100.0;


//pin constants
const int GEARBOX_RIGHT_A = 2;
const int GEARBOX_RIGHT_B = 3;
const int OUTPUT_ENABLE = 4;
const int JETSON_ENABLE = 8;
const int NEUTRAL = 470;

enum class Address : int16_t
{
    TREAD_LEFT = 0,
    TREAD_RIGHT = 1,
    ARM_LOWER = 4,
    ARM_UPPER = 5,
    ARM_SCOOP = 6,
    ARM_TURNTABLE = 7,
    BIN_LEFT = 2,
    BIN_RIGHT = 3
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
    pinMode(JETSON_ENABLE, OUTPUT);
    digitalWrite(JETSON_ENABLE, LOW);
    delay(500);
    digitalWrite(JETSON_ENABLE, HIGH);
    pinMode(OUTPUT_ENABLE, OUTPUT);
    digitalWrite(OUTPUT_ENABLE, HIGH);
    for (auto& val : pwm_values)
      val = NEUTRAL;

    nh.initNode();
    nh.advertise(arduino);
    nh.subscribe(motor_subscriber);
    pwm.begin();
    pwm.setPWMFreq(80);  // This is the maximum PWM frequency
    setAddress(Address::TREAD_LEFT, 0, FULL_DELTA);
    setAddress(Address::TREAD_RIGHT, 0, FULL_DELTA);
    setAddress(Address::ARM_TURNTABLE, 0, FULL_DELTA);
    setAddress(Address::ARM_LOWER, 0, FULL_DELTA);
    setAddress(Address::ARM_UPPER, 0, FULL_DELTA);
    setAddress(Address::ARM_SCOOP, 0, FULL_DELTA);
    setAddress(Address::BIN_LEFT, 0, FULL_DELTA);
    setAddress(Address::BIN_RIGHT, 0, FULL_DELTA);
}

void loop()
{
    arduino_reading.tread_right_vel = gearbox_right.getVelocity()/GEARBOX_MPR;
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
        setAddress(Address::TREAD_LEFT, command.tread_left, MAX_DRIVEBASE_DELTA);
        setAddress(Address::TREAD_RIGHT, command.tread_right, MAX_DRIVEBASE_DELTA);
        setAddress(Address::ARM_TURNTABLE, command.arm_turntable, MAX_ARM_DELTA);
        setAddress(Address::ARM_LOWER, command.arm_lower, MAX_ARM_DELTA);
        setAddress(Address::ARM_UPPER, command.arm_upper, MAX_ARM_DELTA);
        setAddress(Address::ARM_SCOOP, command.arm_scoop, MAX_ARM_DELTA);
        setAddress(Address::BIN_LEFT, command.bin_left, MAX_ARM_DELTA);
        setAddress(Address::BIN_RIGHT, command.bin_right, MAX_ARM_DELTA);
    }
    else
    {
      	digitalWrite(OUTPUT_ENABLE, HIGH);
        setAddress(Address::TREAD_LEFT, 0, FULL_DELTA);
        setAddress(Address::TREAD_RIGHT, 0, FULL_DELTA);
        setAddress(Address::ARM_TURNTABLE, 0, FULL_DELTA);
        setAddress(Address::ARM_LOWER, 0, FULL_DELTA);
        setAddress(Address::ARM_UPPER, 0, FULL_DELTA);
        setAddress(Address::ARM_SCOOP, 0, FULL_DELTA);
        setAddress(Address::BIN_LEFT, 0, FULL_DELTA);
        setAddress(Address::BIN_RIGHT, 0, FULL_DELTA);
    }
}

/*
 * sets a pwm output scaled between -1 and 1. Limits output change to 5% per cycle.
 */
void setAddress(const Address &addr, float val, float max_delta)
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
    if (((delta >= 0) ? delta : -delta) > max_delta)
        pwm_signal = pwm_values[address] + (sign*max_delta);
    
    pwm_values[address] = pwm_signal;
    pwm.setPWM(address, 0, pwm_signal);
}
