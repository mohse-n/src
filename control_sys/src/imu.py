#!/usr/bin/env python

import rospy
import FaBo9Axis_MPU9250
from control_sys.msg import ImuData

mpu9250 = FaBo9Axis_MPU9250.MPU9250()
rospy.init_node('imu')

pub = rospy.Publisher('imu_data',ImuData, queue_size = 1)
out = ImuData()
#out.orientation_covariance[0] = -1
#out.header.frame_id = 'imu'
rate = rospy.Rate(200)

while not rospy.is_shutdown():
	
		accel = mpu9250.readAccel()
		gyro = mpu9250.readGyro()

		out.header.stamp = rospy.Time.now()

		out.quat.x = (accel['x'] - 0.01) * 9.81
		out.quat.y = (accel['y'] - 0.04) * 9.81
		out.quat.z = (accel['z'] - 0.02) * 9.81

		out.vel.x = (gyro['x'] + 1.3) * 0.01745329251
		out.vel.y = (gyro['y'] - 0.5) * 0.01745329251
		out.vel.z = gyro['z'] * 0.01745329251

		pub.publish(out)
		rate.sleep()
