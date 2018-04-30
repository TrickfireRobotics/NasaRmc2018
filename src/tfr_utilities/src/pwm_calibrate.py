#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from tfr_msgs.msg import PwmCommand

def talker():
    pub = rospy.Publisher('/motor_output', PwmCommand, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    msg = PwmCommand()
    msg.enabled = True
    
    val = -1.0
    while not rospy.is_shutdown() and val < 1.001:
        msg.arm_upper = val
        rospy.loginfo("val {}".format(val))
        val += 0.001
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
