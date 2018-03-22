#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ros.h>
#include <tfr_msgs/ArduinoReading.h>
#include <quadrature.h>

ros::NodeHandle nh;

//encoder level constants
const double GEARBOX_CPR = 4096; //pulse per revolution
//TODO note this rev/m is acually 0.87, but I had to step it up to make testing work
const double GEARBOX_RPM = 60; //revolutions per meter

//pin constants
const int GEARBOX_LEFT_A = 2;
const int GEARBOX_LEFT_B = 3;

struct potentiometer
{
    const static float ADC_RANGE=0.0001875;
    float extension_range; //the max range the potentiometer can be extended
    float max_voltage; //the max voltage for the potentiometer
    float getExtension(int16_t val)
    {
        return val* ADC_RANGE/max_voltage; 
    }
};

potentiometer pots[6] = 
    {
        {0.15, 4.88},
        {0.15, 4.88},
        {0.15, 4.88},
        {0.15, 4.88},
        {0.15, 4.88},
        {0.15, 4.88}
    };


//encoders
Quadrature gearbox_left(GEARBOX_CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::ArduinoReading arduinoReading;
ros::Publisher arduino("arduino", &arduinoReading);

//potentiometers
Adafruit_ADS1115 ads1115;

void setup()
{
    nh.initNode();
    nh.advertise(arduino);
    ads1115.begin();
}

void loop()
{
    arduinoReading.tread_right_vel = arduinoReading.tread_left_vel = gearbox_left.getVelocity()/GEARBOX_RPM;
    gearbox_left.getVelocity()/GEARBOX_RPM;
    //used to test latency
    gearbox_left.getVelocity()/GEARBOX_RPM;
    arduinoReading.arm_turntable_pos = 0;

    //TODO hook up other encoders
    ads1115.startADC_SingleEnded(0);
    delay(8);
    arduinoReading.arm_lower_left_pos = pots[0].getExtension(ads1115.collectADC_SingleEnded());
    arduinoReading.arm_lower_right_pos = 0;
    ads1115.collectADC_SingleEnded();

    ads1115.startADC_SingleEnded(0);
    ads1115.startADC_SingleEnded(1);
    ads1115.startADC_SingleEnded(2);
    delay(8);
    arduinoReading.arm_scoop_pos = arduinoReading.arm_upper_pos = 0;
    ads1115.collectADC_SingleEnded();
    ads1115.collectADC_SingleEnded();

    ads1115.startADC_SingleEnded(1);
    ads1115.startADC_SingleEnded(2);
    delay(8);
    arduinoReading.bin_right_pos = arduinoReading.bin_left_pos = 0;
    ads1115.collectADC_SingleEnded();
    ads1115.collectADC_SingleEnded();

    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    arduino.publish(&arduinoReading);
}
