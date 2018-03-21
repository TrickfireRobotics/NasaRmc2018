#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ros.h>
#include <tfr_msgs/EncoderReading.h>
#include <tfr_msgs/PotentiometerReading.h>
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

const potentiometer pots[6] = 
    {
        {0.15, 5.0},
        {0.15, 5.0},
        {0.15, 5.0},
        {0.15, 5.0},
        {0.15, 5.0},
        {0.15, 5.0}
    };


//encoders
Quadrature gearbox_left(GEARBOX_CPR, GEARBOX_LEFT_A, GEARBOX_LEFT_B);
tfr_msgs::EncoderReading encoderReading;

ros::Publisher encoders("encoders", &encoderReading);

//potentiometers
Adafruit_ADS1115 ads1115;
tfr_msgs::PotentiometerReading potentiometerReading;
ros::Publisher potentiometers("potentiometers", &potentiometerReading);

void setup()
{
    nh.initNode();
    nh.advertise(encoders);
    nh.advertise(potentiometers);
    ads1115.begin();
}

void loop()
{
    encoderReading.right_vel = encoderReading.left_vel = gearbox_left.getVelocity()/GEARBOX_RPM;
    gearbox_left.getVelocity()/GEARBOX_RPM;
    //used to test latency
    gearbox_left.getVelocity()/GEARBOX_RPM;
    encoderReading.turntable_pos = 0;

    //TODO hook up other encoders
    ads1115.startADC_SingleEnded(0);
    delay(8);
    potentiometerReading.pot_0 = pots[0].getExtension(ads1115.collectADC_SingleEnded());
    potentiometerReading.pot_1 = 0;
    ads1115.collectADC_SingleEnded();

    ads1115.startADC_SingleEnded(0);
    ads1115.startADC_SingleEnded(1);
    ads1115.startADC_SingleEnded(2);
    delay(8);
    potentiometerReading.pot_2 = potentiometerReading.pot_3 = 0;
    ads1115.collectADC_SingleEnded();
    ads1115.collectADC_SingleEnded();

    ads1115.startADC_SingleEnded(1);
    ads1115.startADC_SingleEnded(2);
    delay(8);
    potentiometerReading.pot_4 = potentiometerReading.pot_5 = 0;
    ads1115.collectADC_SingleEnded();
    ads1115.collectADC_SingleEnded();

    nh.spinOnce(); //I know we don't have any callbacks, but the libary needs this call
    encoders.publish(&encoderReading);
    potentiometers.publish(&potentiometerReading);
}
