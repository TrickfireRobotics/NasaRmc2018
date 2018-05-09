#ifndef QUADRATURE_H
#define QUADRATURE_H
#include <Encoder.h>
#include <limits.h>

/*
  Class for efficiently reading quadrature encoder data, instantiate, and
  call getVelocity in the main loop. This is designed to be used in a system 
  who only has one job, to read these quadrature encoders
  */
class VelocityQuadrature
{
  public:
    VelocityQuadrature(int cpr, int a_pin, int b_pin):
    CPR{cpr}, data{}, encoder{a_pin,b_pin} {}
  /*
    Gives the velocity of controller in shaftrevloutions/sec, since the last time
    getVelocity was called, should be called often to ensure accuracy, provides interrupt safety
  */
  double getVelocity()
  {
    //interrupt safety is handled by lower level library
    data.p_1 = encoder.read();
    
    //handle overflow by recentering data before we need to
    if ( data.p_1 > 0.8 * INT_MAX || data.p_1 < 0.8 * INT_MIN) 
    {
        encoder.write(0);
        data.p_1 = data.p_1-data.p_0;
        data.p_0 = 0;
    }
    
    //time will only overflow every 50 days of consecutive operation, non issue
    data.t_1 = millis()*1e-3; 

    //calculate the velocity
    double velocity = (data.p_1 - data.p_0)/((double)(data.t_1-data.t_0)*CPR);
    
    data.p_0 = data.p_1;
    data.t_0 = data.t_1;
    
    return velocity;
  }

  private:

    //structures
    struct EncoderData
    {
      int32_t p_0 = 0;
      double t_0 = 0;
      int32_t p_1 = 0;
      double t_1 = 0;
    };

    const int CPR;

    EncoderData data;

    Encoder encoder;
    
};
/*
  Class for efficiently reading quadrature encoder data, instantiate, and
  call getVelocity in the main loop. This is designed to be used in a system 
  who only has one job, to read these quadrature encoders
  */
class PositionQuadrature
{
  public:
    PositionQuadrature(int cpr, int a_pin, int b_pin):
    CPR{cpr}, encoder{a_pin,b_pin} {}
  /*
   * absolute position in radians
  */
  double getPosition()
  {
    return encoder.read()/CPR;
  }

  private:

    const int CPR;

    Encoder encoder;
    
};

#endif
