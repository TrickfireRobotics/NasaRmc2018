#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ros.h>
#include <tfr_msgs/ArduinoAReading.h>
#include <quadrature.h>

ros::NodeHandle nh;

//encoder level constants
const double GEARBOX_CPR = 4096;
const double TURNTABLE_CPR = 28; 
const double GEARBOX_MPR = 2*3.14*0.15; 
const double TURNTABLE_RPR = (1/188.0)*(15.0/72.0)*2.0*3.15159; 

//pin constants
const int GEARBOX_LEFT_A = 2;
const int GEARBOX_LEFT_B = 3;
const int TURNTABLE_A = 18;
const int TURNTABLE_B = 19;

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
        float new_estimate = 0.0174533*(m*val + b);
        estimate += (new_estimate - estimate)/4;
        return estimate;
    }
};

enum Potentiometers
{
    //used for array indexes
    ARM_LOWER,
    ARM_UPPER,
    ARM_SCOOP,
    BIN_LEFT,
    BIN_RIGHT
};

Potentiometer pots []
{
  Potentiometer{0.0071, 4.1523},    //ARM_LOWER
  Potentiometer{0.0149, 17.319},    //ARM_UPPER
  Potentiometer{0.0207, -269.57},    //ARM_SCOOP
  Potentiometer{0.00346, -23.672},            //BIN_LEFT TODO
  Potentiometer{0.00348, -23.882}             //BIN_RIGHT TODO
};


PositionQuadrature turntable(TURNTABLE_CPR, TURNTABLE_A, TURNTABLE_B); 

//encoders
VelocityQuadrature gearbox_left(GEARBOX_CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
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

Adafruit_ADS1115 ads1115_b(0x49);
void setup()
{
    nh.initNode();
    nh.advertise(arduino);
    ads1115_a.begin();
}

void loop()
{
    arduinoReading.tread_left_vel = gearbox_left.getVelocity() * GEARBOX_MPR;
    arduinoReading.arm_turntable_pos = turntable.getPosition()  * TURNTABLE_RPR;

    ads1115_a.startADC_SingleEnded(2);
    ads1115_b.startADC_SingleEnded(0);
    delay(8);
    arduinoReading.arm_upper_pos = pots[ARM_UPPER].getPosition(ads1115_a.collectADC_SingleEnded());
    arduinoReading.bin_left_pos = pots[BIN_LEFT].getPosition(ads1115_b.collectADC_SingleEnded());
    nh.spinOnce(); 

    ads1115_a.startADC_SingleEnded(1);
    ads1115_b.startADC_SingleEnded(1);
    delay(8);
    arduinoReading.arm_scoop_pos = pots[ARM_SCOOP].getPosition(ads1115_a.collectADC_SingleEnded());
    arduinoReading.bin_right_pos = pots[BIN_RIGHT].getPosition(ads1115_b.collectADC_SingleEnded());
    nh.spinOnce();

    ads1115_a.startADC_SingleEnded(3);
    delay(8);
    arduinoReading.arm_lower_pos = pots[ARM_LOWER].getPosition(ads1115_a.collectADC_SingleEnded());
    nh.spinOnce();

    arduino.publish(&arduinoReading);
}
