#!/usr/bin/env python
#test code to never be seen again
import rospy
from tfr_msgs.msg import ArduinoReading

def talker():
    pub = rospy.Publisher('/arduino', ArduinoReading, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    reading = ArduinoReading()
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.loginfo("fwd")
    reading.tread_left_vel = 1;
    reading.tread_right_vel = 1;
    pub.publish(reading)
    rate.sleep()
    rospy.loginfo("fwdleft")
    reading.tread_left_vel = 0.5;
    reading.tread_right_vel = 1;
    pub.publish(reading)
    rate.sleep()
    rospy.loginfo("fwdright")
    reading.tread_left_vel = 1;
    reading.tread_right_vel = 0.5;
    pub.publish(reading)
    rate.sleep()
    rospy.loginfo("back")
    reading.tread_left_vel = -1;
    reading.tread_right_vel = -1;
    pub.publish(reading)
    rate.sleep()
    rospy.loginfo("backleft")
    reading.tread_left_vel = -0.5;
    reading.tread_right_vel = -1;
    pub.publish(reading)
    rate.sleep()
    rospy.loginfo("backright")
    reading.tread_left_vel = -1;
    reading.tread_right_vel = -0.5;
    pub.publish(reading)
    rate.sleep()
    rospy.loginfo("right")
    reading.tread_left_vel = -1;
    reading.tread_right_vel = 1;
    pub.publish(reading)
    rate.sleep()
    rospy.loginfo("left")
    reading.tread_left_vel = 1;
    reading.tread_right_vel = -1;
    pub.publish(reading)
    rate.sleep()




if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
