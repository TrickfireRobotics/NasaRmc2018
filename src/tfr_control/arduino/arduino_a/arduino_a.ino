#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ros.h>
#include <tfr_msgs/ArduinoAReading.h>
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
    Potentiometer(float slope, float intercept) :
      m{slope}, b{intercept}, estimate{0.0} {}

    float m{}; //the slope of the linear graph
    float b{}; //the y intercept of the linear
    float estimate{}; //uses exponential smoothing 

    float getPosition(uint16_t val)
    {
        // 4 is the coefficient that represents how much we trust the estimate
        // vs how much we trust the newest reading. The lower this coefficient, the
        // more noise we have. The higher the coefficient, the more polluted our 
        // result becomes with erroneous noise data. 4 seems to be a good middle
        // ground.
        estimate += (m*val + b - estimate)/4;
        return estimate;
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
  Potentiometer{0.0075, -26.482},    //ARM_LOWER
  Potentiometer{0.0144, -173.77},    //ARM_UPPER
  Potentiometer{0.0214, -239.97},    //ARM_SCOOP
  Potentiometer{0.0, 0.0},            //BIN_LEFT TODO
  Potentiometer{0.0, 0.0}             //BIN_RIGHT TODO
};


//encoders
VelocityQuadrature gearbox_left(CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::ArduinoAReading arduinoReading;
ros::Publisher arduino("arduino_a", &arduinoReading);

//potentiometers
/*
ADC Adressing 

ads1115_a => 
  channel 0 : ARM_UPPER
  channel 1 : ARM_SCOOP
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
    arduinoReading.arm_upper_pos = pots[ARM_UPPER].getPosition(ads1115_a.collectADC_SingleEnded());
    nh.spinOnce(); 

    ads1115_a.startADC_SingleEnded(1);
    delay(8);
    arduinoReading.arm_scoop_pos = pots[ARM_SCOOP].getPosition(ads1115_a.collectADC_SingleEnded());
    nh.spinOnce();

    ads1115_a.startADC_SingleEnded(2);
    delay(8);
    arduinoReading.arm_lower_pos = pots[ARM_LOWER].getPosition(ads1115_a.collectADC_SingleEnded());
    nh.spinOnce();

    arduino.publish(&arduinoReading);
}
