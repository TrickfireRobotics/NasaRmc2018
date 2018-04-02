
#include <Encoder.h>
#include <Wire.h>
#include <ros.h>
#include <tfr_msgs/ArduinoBReading.h>
#include <tfr_utilities/control_code.h>
#include <quadrature.h>

ros::NodeHandle nh;

//encoder level constants
const double CPR = 4096; //pulse per revolution
const double GEARBOX_RPM = 3.3314; 
//TODO note this rev/rev is acually 0.208, but I had to step it up to make testing work
const double TURNTABLE_RPR = 60;

//pin constants
const int GEARBOX_RIGHT_A = 2;
const int GEARBOX_RIGHT_B = 3;







//encoders
VelocityQuadrature gearbox_right(CPR, GEARBOX_RIGHT_A, GEARBOX_RIGHT_B);
//TODO Quadrature turntable(CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::ArduinoBReading arduinoReading;
ros::Publisher arduino("arduino_b", &arduinoReading);


void setup()
{
    nh.initNode();
    nh.advertise(arduino);
}

void loop()
{
    arduinoReading.tread_right_vel = gearbox_right.getVelocity()/GEARBOX_RPM;
    //TODO    arduinoReading.arm_turntable_pos = turntable.getVelocity()/TURNTABLE_RPR;
    delay(8);
    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    delay(8);
    nh.spinOnce();
    delay(8);
    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    arduino.publish(&arduinoReading);
}
