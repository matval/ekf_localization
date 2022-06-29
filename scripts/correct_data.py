#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import signal, sys, time, math
import tf

class CorrectData():
    def __init__(self):
        sub_imu_topic = rospy.get_param('~sub_imu_topic', 'imu')
        self.sub_imu = rospy.Subscriber(sub_imu_topic, Imu, self.sub_imu_cb)

        pub_imu_topic = rospy.get_param('~pub_imu_topic', 'imu_corrected')
        self.pub_imu = rospy.Publisher(pub_imu_topic, Imu, queue_size=1)

        self.checkSignParams()

    def sub_imu_cb(self, msg):
        fix_imu = Imu()
        fix_imu.header.stamp = msg.header.stamp

        fix_imu.angular_velocity.x = self.sign_gyro_vroll*msg.angular_velocity.y #it was inverted with y in ts_rosbridge_client.py
        fix_imu.angular_velocity.y = self.sign_gyro_vpitch*msg.angular_velocity.x
        fix_imu.angular_velocity.z = self.sign_gyro_vyaw*msg.angular_velocity.z
        fix_imu.linear_acceleration.x = self.sign_accel_x*msg.linear_acceleration.x 
        fix_imu.linear_acceleration.y = self.sign_accel_y*msg.linear_acceleration.y 
        fix_imu.linear_acceleration.z = self.sign_accel_z*msg.linear_acceleration.z 

        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = self.sign_roll*euler[0]
        pitch = self.sign_pitch*euler[1]
        yaw = self.sign_yaw*euler[2]

        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        fix_imu.orientation.x = q[0]
        fix_imu.orientation.y = q[1]
        fix_imu.orientation.z = q[2]
        fix_imu.orientation.w = q[3]
        self.pub_imu.publish(fix_imu)


    def checkSignParams(self):
        self.sign_accel_x  = 1
        self.sign_accel_y  = 1
        self.sign_accel_z  = 1
        self.sign_gyro_vyaw = 1
        self.sign_gyro_vpitch = 1
        self.sign_gyro_vroll = 1
        self.sign_yaw = 1
        self.sign_pitch = 1
        self.sign_roll = 1

        invert_accel_x = rospy.get_param('~invert_accel_x', False)
        if invert_accel_x == 1:
            self.sign_accel_x = -1
        invert_accel_y = rospy.get_param('~invert_accel_y', False)
        if invert_accel_y == 1:
            self.sign_accel_y = -1
        invert_accel_z = rospy.get_param('~invert_accel_z', False)
        if invert_accel_z == 1:
            self.sign_accel_z = -1
        invert_gyro_vyaw = rospy.get_param('~invert_gyro_vyaw', False)
        if invert_gyro_vyaw == 1:
            self.sign_gyro_vyaw = -1
        invert_gyro_vpitch = rospy.get_param('~invert_gyro_vpitch', False)
        if invert_gyro_vpitch == 1:
            self.sign_gyro_vpitch = -1
        invert_gyro_vroll = rospy.get_param('~invert_gyro_vroll', False)
        if invert_gyro_vroll == 1:
            self.sign_gyro_vroll = -1
        invert_yaw = rospy.get_param('~invert_yaw', False)
        if invert_yaw == 1:
            self.sign_yaw = -1
        invert_pitch = rospy.get_param('~invert_pitch', False)
        if invert_pitch == 1:
            self.sign_pitch = -1
        invert_roll = rospy.get_param('~invert_roll', False)
        if invert_roll == 1:
            self.sign_roll = -1
        print('sign accel x', self.sign_accel_x, 'y', self.sign_accel_y, 'z', self.sign_accel_z)
        print('sign gyro vyaw', self.sign_gyro_vyaw, 'vpitch', self.sign_gyro_vpitch, 'vroll', self.sign_gyro_vroll)
        print('sign yaw', self.sign_yaw, 'pitch', self.sign_pitch, 'roll', self.sign_roll)

def signal_handler(sig, frame):
    print('plot_gps_node You pressed Ctrl+C!')
    rospy.signal_shutdown('Ctrl+C')
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('plot_gps_node', disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)

    correct = CorrectData()

    rospy.spin()

