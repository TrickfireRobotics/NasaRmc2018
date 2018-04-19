
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
const double GEARBOX_MPR = 0.88; 

//pin constants
const int GEARBOX_LEFT_A = 2;
const int GEARBOX_LEFT_B = 3;






struct Potentiometer
{
   const static float ADC_RANGE=0.0001875;
   float max_voltage; //the max voltage for the potentiometer
   float min_angle;
   float max_angle;
   float getPosition(int16_t val)
   {
       return min_angle + (max_angle - min_angle) * val* ADC_RANGE/max_voltage; 
   }
};

enum Potentiometers
{
    //used for array indexes
    ARM_LOWER_LEFT,
    ARM_LOWER_RIGHT,
    ARM_UPPER,
    ARM_SCOOP,
    BIN_LEFT,
    BIN_RIGHT
};

Potentiometer pots []
{
  {4.88, tfr_utilities::JointAngle::ARM_LOWER_MIN, tfr_utilities::JointAngle::ARM_LOWER_MAX},      //ARM_LOWER_LEFT
  {4.88, tfr_utilities::JointAngle::ARM_LOWER_MIN, tfr_utilities::JointAngle::ARM_LOWER_MAX},      //ARM_LOWER_RIGHT
  {4.88, tfr_utilities::JointAngle::ARM_UPPER_MIN, tfr_utilities::JointAngle::ARM_UPPER_MAX},      //ARM_UPPER
  {4.88, tfr_utilities::JointAngle::ARM_SCOOP_MIN, tfr_utilities::JointAngle::ARM_SCOOP_MAX},      //ARM_SCOOP
  {4.88, tfr_utilities::JointAngle::BIN_MIN, tfr_utilities::JointAngle::BIN_MAX},      //BIN_LEFT
  {4.88, tfr_utilities::JointAngle::BIN_MIN, tfr_utilities::JointAngle::BIN_MAX}       //BIN_RIGHT
};


//encoders
VelocityQuadrature gearbox_left(CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::ArduinoAReading arduinoReading;
ros::Publisher arduino("arduino_a", &arduinoReading);

//potentiometers
/*
ADC Adressing 

ads1115_a => 
  channel 0 : ARM_LOWER_LEFT
  channel 1 : ARM_LOWER_RIGHT
  channel 2 : ARM_UPPER
  channel 3 : UNUSED
ads1115_b => 
  channel 0 : ARM_SCOOP
  channel 1 : BIN_LEFT
  channel 2 : BIN_RIGHT
  channel 3 : UNUSED
*/

Adafruit_ADS1115 ads1115_a;
//TODO Adafruit_ADS1115 ads1115_b;

void setup()
{
    nh.initNode();
    nh.advertise(arduino);
    ads1115_a.begin();
//TODO    ads1115_b.begin();
}

void loop()
{
    arduinoReading.tread_left_vel = -gearbox_left.getVelocity()/GEARBOX_MPR;


    ads1115_a.startADC_SingleEnded(0);
//TODO    ads1115_b.startADC_SingleEnded(0);
    delay(8);
    //TODO Get real values here
    arduinoReading.arm_lower_right_pos = arduinoReading.arm_lower_left_pos = pots[ARM_LOWER_LEFT].min_angle;
    arduinoReading.arm_upper_pos = pots[ARM_UPPER].min_angle;
    arduinoReading.arm_scoop_pos = pots[ARM_SCOOP].min_angle;
    arduinoReading.bin_right_pos = arduinoReading.bin_left_pos = pots[BIN_LEFT].getPosition(ads1115_a.collectADC_SingleEnded());
    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
//TODO    arduinoReading.arm_scoop_pos = pots[ARM_SCOOP].getPosition(ads1115_b.collectADC_SingleEnded());



//TODO    ads1115_a.startADC_SingleEnded(1);
//TODO    ads1115_b.startADC_SingleEnded(1);
          delay(8);
          nh.spinOnce();
//TODO    arduinoReading.arm_lower_right = pots[ARM_LOWER_RIGHT].getPosition(ads1115_a.collectADC_SingleEnded());
//TODO    arduinoReading.bin_left_pos = pots[BIN_LEFT].getPosition(ads1115_b.collectADC_SingleEnded());

//TODO    ads1115_a.startADC_SingleEnded(2);
//TODO    ads1115_b.startADC_SingleEnded(2);
          delay(8);
//TODO    arduinoReading.arm_upper = pots[ARM_UPPER].getPosition(ads1115_a.collectADC_SingleEnded());
//TODO    arduinoReading.bin_right = pots[BIN_RIGHT].getPosition(ads1115_b.collectADC_SingleEnded());

    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    arduino.publish(&arduinoReading);
}
