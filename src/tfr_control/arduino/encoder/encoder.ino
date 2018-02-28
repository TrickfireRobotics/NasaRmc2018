#include <Encoder.h>

#include <ros.h>
#include <tfr_msgs/EncoderReading.h>

#include <quadrature.h>

ros::NodeHandle nh;



//encoder level constants
const double GEARBOX_CPR = 4096; //pulse per revolution
const double GEARBOX_RPM = 0.876; //revolutions per meter

//pin constants
const int GEARBOX_LEFT_A = 2;
const int GEARBOX_LEFT_B = 3;

//encoders
Quadrature gearbox_left(GEARBOX_CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);

//algorithm constants
const float rate(1/50.0);
double left_velocity = 0;

//note the method signature get's mangled by the AVR compiler
/*I have literally no control over what this returns*/
void readEncoders(const tfr_msgs::EncoderReadingRequest &req,
                   tfr_msgs::EncoderReadingResponse &res)
{
    res.left_vel = left_velocity;
    //TODO hook up right encoder
    //TODO hook up the position of the turntable
}

ros::ServiceServer<tfr_msgs::EncoderReading::Request, tfr_msgs::EncoderReading::Response> 
      server("read_encoders", readEncoders);
void setup()
{
    nh.initNode();
    nh.advertiseService(server);
    nh.getParam("~rate", &rate);
}

void loop()
{
    left_velocity = gearbox_left.getVelocity()/GEARBOX_RPM;
    //TODO hook up right encoder
    nh.spinOnce();
    delay(rate);
}
