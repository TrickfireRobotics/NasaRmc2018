
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ros.h>
#include <tfr_msgs/ArduinoAReading.h>
#include <tfr_utilities/control_code.h>
#include <quadrature.h>

ros::NodeHandle nh;

//encoder level constants
const double CPR = 4096; //pulse per revolution
const double GEARBOX_MPR = 3.33; 

//pin constants
const int GEARBOX_LEFT_A = 2;
const int GEARBOX_LEFT_B = 3;




/*Linear potentiometer, values gained from empirical measurement*/
struct Potentiometer
{
    float m{}; //the slope of the linear graph
    float b{}; //the y intercept of the linear

    float getPosition(uint16_t val)
    {
        return m*x + b
    }
};

enum Potentiometers
{
    //used for array indexes
    ARM_SCOOP,
    ARM_UPPER,
    ARM_LOWER,
    BIN_LEFT,
    BIN_RIGHT
};

Potentiometer pots []
{
  {-0.0075, 116.51},    //ARM_LOWER
  {-0.0144, 353.77},    //ARM_UPPER
  {-0.0214, 328.97},    //ARM_SCOOP
  {0.0,0.0},            //BIN_LEFT TODO
  {0.0,0.0}             //BIN_RIGHT TODO
};


//encoders
VelocityQuadrature gearbox_left(CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::ArduinoAReading arduinoReading;
ros::Publisher arduino("arduino_a", &arduinoReading);

//potentiometers
/*
ADC Adressing 

ads1115_a => 
  channel 0 : ARM_SCOOP
  channel 1 : ARM_UPPER
  channel 2 : ARM_LOWER
  channel 3 : UNUSED
ads1115_b => 
  channel 0 : BIN_LEFT
  channel 1 : BIN_RIGHT
  channel 1 : UNUSED
  channel 3 : UNUSED
*/

Adafruit_ADS1115 ads1115_a;

void setup()
{
    nh.initNode();
    nh.advertise(arduino);
    ads1115_a.begin();
}

void loop()
{
    arduinoReading.tread_left_vel = -gearbox_left.getVelocity()/GEARBOX_MPR;

    ads1115_a.startADC_SingleEnded(0);
    delay(8);
    arduinoReading.arm_scoop_pos = pots[ARM_SCOOP].getPosition(ads1115_a.collectADC_SingleEnded());
    nh.spinOnce(); 

    ads1115_a.startADC_SingleEnded(1);
    delay(8);
    arduinoReading.arm_upper_pos = pots[ARM_UPPER].getPosition(ads1115_a.collectADC_SingleEnded());
    nh.spinOnce();

    ads1115_a.startADC_SingleEnded(2);
    delay(8);
    arduinoReading.arm_lower_pos = pots[ARM_LOWER].getPosition(ads1115_a.collectADC_SingleEnded());
    nh.spinOnce();

    arduino.publish(&arduinoReading);
}
