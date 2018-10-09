Notes and documentation on the robot arm.
So the robot arm has 3 linear actuators.
	These extend when they receive a positive voltage, and contract when the receive a negative voltage.
		The greater the voltage, the faster they move.
		
We have a potentiometer (it must be multiple?) attached to the arm.
	There's a 5v reference voltage line there as well.
	The voltage on the potentiometers, along with the reference voltage, are fed into a hardware ADC.
		Then a digital output from there goes to an Arduino.
		
Arduinos:
	One arduino is reading in all the robot inputs from the different motor encoders (which are digital signals)
	The other Arduino, through a separate board that we use just so that we have more pins, currently sends PWM signals out to all of the motors.
		We are MOVING so that ALL outgoing messages to the motors from the Arduino will be done over CAN.
		The inputs that we are reading in will still remain in the previous format. i.e. Digital signals.
			Uhhh actually I am not sure if the encoders on the arm will continue to be provided to us as digital signals and not as CAN messages.
		There is an issue with using PWM to control actuators over long distance.
		For the dumping mechanism, we need to make sure that the two linear actuators remain in sync.
			John suggested to use PID controller, and that handling that will be easy.
		For controlling the actuators using CAN messages:
			None of us know exactly how to do it.
			Reason for switch to CAN is because it's cool and lots of cool projects do it.
				However, it also lowers the latency when handling high-priority interrupts, since in the synchronous CAN bus protocol, high priority messages beat out low-priority ones as data is sent on the wire.
					But of course, that depends on all the other CAN message senders behaving appropriately.

Note to self, I don't know what rotary/motor encoders look like.
The silver/transparent-looking cables go out to the motor controllers.
					
The Jetson:
	The jetson currently does any of the actual reading from the encoders, or sending of PWM signals to the motor controllers.
	The jetson was freed up to do SLAM.
		But the Jetson is also running it's NVIDIA flavor of linux, and is running ROS and doing all kinds of ROS stuff.
