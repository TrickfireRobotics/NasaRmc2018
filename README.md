# NasaRmc2018 [![Build Status](http://40.65.120.141:8080/job/Jenkins%20Build%20-%20NasaRmc2018/badge/icon)](http://40.65.120.141:8080/job/Jenkins%20Build%20-%20NasaRmc2018/)

This is the primary codebase for all of the TrickFire Robotics Nasa RMC 2018 competition entry.

# Installation
Installing the code requres, in addition to the standard `ros-kinetic-desktop-full` installation, these packages (available through `apt`):
 * `ros-kinetic-controller-manager`
 * `ros-kinetic-move-base`
 * `ros-kinetic-hardware-interface`
 * `ros-kinetic-effort-controllers`
 * `ros-kinetic-joint-state-controller`
 * `ros-kinetic-joint-trajectory-controller`
 * `libsfml-dev`
 * `ros-kinetic-gps-common`
 * `ros-kinetic-move-base`
 * `ros-kinetic-moveit`
 * `ros-kinetic-moveit-ros`
 * `ros-kinetic-cv-camera`
 * `ros-kinetic-vision-opencv`
 * `ros-kinetic-rosserial`
 * `ros-kinetic-rosserial-arduino`
 * `ros-kinetic-rqt`
 * `ros-kinetic-rqt-common-plugins`
 * `ros-kinetic-rqt-robot-plugins`
 * `qtbase5-dev`
 * `ros-kinetic-ros-control`
 * `ros-kinetic-ros-controllers`
 * `libi2c-dev`
 * `libgtest-dev`

# Arduino Libraries
Some of our control software is installed on arduino. Modifications require the Encoder library to be installed. https://www.pjrc.com/teensy/td_libs_Encoder.html

