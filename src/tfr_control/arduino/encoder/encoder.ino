#include <Encoder.h>
#include <ros.h>
#include <tfr_msgs/EncoderReading.h>
#include <quadrature.h>

ros::NodeHandle nh;

//encoder level constants
const double GEARBOX_CPR = 4096; //pulse per revolution
//TODO note this rev/m is acually 0.87, but I had to step it up to make testing work
const double GEARBOX_RPM = 60; //revolutions per meter

//pin constants
const int GEARBOX_LEFT_A = 2;
const int GEARBOX_LEFT_B = 3;

//encoders
Quadrature gearbox_left(GEARBOX_CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::EncoderReading reading;

//algorithm constants
const float rate(1/50.0);

ros::Publisher encoders("encoders", &reading);

void setup()
{
    nh.initNode();
    nh.advertise(encoders);
    nh.getParam("~rate", &rate);
}

void loop()
{
    reading.left_vel = gearbox_left.getVelocity()/GEARBOX_RPM;
    //TODO hook up other encoders

    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    encoders.publish(&reading);
    delay(rate);
}
