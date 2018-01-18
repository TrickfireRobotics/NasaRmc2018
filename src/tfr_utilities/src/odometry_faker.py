#! /usr/bin/env python
import rospy
from math import sin, cos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Quaternion, Vector3
from itertools import izip
import tf


class OdometryFaker(object):
    """Rough and tumble odometry estimator, based on xsens imu"""
    covariance = [ 1e-1,   0,   0,   0,   0,   0,
                      0,1e-1,   0,   0,   0,   0,
                      0,   0,1e-1,   0,   0,   0,
                      0,   0,   0,1e-1,   0,   0,
                      0,   0,   0,   0,1e-1,   0,
                      0,   0,   0,   0,   0,1e-1]
    
    def __init__(self):
        rospy.Subscriber("/sensors/mti/sensor/imu", Imu, self.imuCallback)
        self.pub = rospy.Publisher("/fake_odom", Odometry, queue_size=5 )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.odomCallback)
        self.odom = None
        self.last_time = rospy.get_time()
        self.position = [0,0,0]
        self.lin_vel = [0,0,0]
        self.lin_acc = [0,0,0]
        self.th = 0


    def getVelocities(self, dt, data):
        acc = vector3ListConversion(data.linear_acceleration)

        out = [(x_1 - x_0)/dt for x_1, x_0 in izip(self.lin_acc, acc)]
        self.linear_acceleration = acc
        return out

    def updateState(self, data):
        dt = (rospy.Time.now() - self.last_time).secs
        self.last_time = rospy.Time.now()
        self.lin_vel = self.getVelocities(dt, data)
        dx = (self.lin_vel[0]*cos(self.th) - self.lin_vel[1]*cos(self.th))*dt
        dy = (self.lin_vel[0]*sin(self.th) - self.lin_vel[1]*cos(self.th))*dt
        self.position[0] += dx
        self.position[1] += dy
        quaternion = (
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.th = euler[2]
        print(self.lin_acc, acc)
        out = [(x_1 - x_0)/dt for x_1, x_0 in izip(self.lin_acc, acc)]
        self.lin_acc = acc
        return out

    def updateState(self, data):
        dt = (rospy.get_time()- self.last_time)
        self.last_time += dt 
        self.lin_vel = self.getVelocities(dt, data)
        quaternion = (  data.orientation.x,
                        data.orientation.y,
                        data.orientation.z,
                        data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.th = euler[2]
        dx = (self.lin_vel[0]*cos(self.th) - self.lin_vel[1]*sin(self.th))*dt
        dy = (self.lin_vel[0]*sin(self.th) + self.lin_vel[1]*cos(self.th))*dt
        print (dt ,dx, dy)
        self.position[0] += dx
        self.position[1] += dy


    def packagePose(self, data):
        out = PoseWithCovariance()
        out.pose.position = listVector3Conversion(self.position)
        out.pose.orientation = data.orientation
        out.covariance = OdometryFaker.covariance
        return out

    def packageTwist(self, data):
        out = TwistWithCovariance()
        out.twist.linear = listVector3Conversion(self.lin_vel)
        out.twist.angular = data.angular_velocity
        out.covariance = OdometryFaker.covariance
        return out


    def imuCallback(self,data):
        """"Makes a super rough estimate of position based on the imu data"""
        odom = Odometry()
        odom.header = data.header
        odom.child_frame_id = "base_footprint"
        self.updateState(data)
        odom.pose =  self.packagePose(data)
        odom.twist = self.packageTwist(data)
        #mark the message as ready
        self.odom = odom

    def odomCallback(self,event):
        if self.odom:
            self.pub.publish(self.odom)
            self.odom = None

def listVector3Conversion(l):
    vec = Vector3()
    vec.x = l[0]
    vec.y = l[1]
    vec.z = l[2]
    return vec

def vector3ListConversion(vec):
    return [vec.x, vec.y, vec.z ]
        

if __name__ == '__main__':
    rospy.init_node("fake_odometry")
    faker = OdometryFaker()
    rospy.spin()
    faker.timer.shutdown()

