# Instructions
To install and work with the arduino software, do it through the arduino ide.

You need:
- The Encoder Library https://www.pjrc.com/teensy/td_libs_Encoder.html
- The Adafruit ads1x15 library https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/arduino-code
- quadrature.h installed in your arduino custom libraries folder


After this use the arduino ide to handle compiling and uploading the files to your board

Since this package relies on our custom msgs, you will need to follow these instructions
to make our headers visible to the avr compiler:
http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages

Note this also requires the CDC -> ACM module be installed on the jetson.
https://github.com/jetsonhacks/installACMModule

If you ever reflash the jetson, really make sure to do both of these steps or you are in a world of hurt.
