# Note
There might be a missing dependency in this package for most ros users. We do not need to use gps data, but the xsens driver marks the ros gps core as a dependency, because the driver can be configured to work with multiple models of sensors. Some of the higher end models in the MTI series contain gps sensors.

Here is how to install the missing package:
'$ sudo apt-get install ros-kinetic-gps-umd'
