#!/usr/bin/env python

import rospy
import pigpio
from control_sys.msg import ImuData


def callback(msg):
		pi.set_PWM_dutycycle(6,msg.quat.x)
		pi.set_PWM_dutycycle(19,msg.quat.y)	
		pi.set_PWM_dutycycle(13,msg.quat.z)
		pi.set_PWM_dutycycle(5,msg.quat.w)

rospy.init_node('motor')
sub = rospy.Subscriber('motor_cmd', ImuData, callback)

pi = pigpio.pi()

#pi.set_PWM_frequency(5,800)
#pi.set_PWM_frequency(6,800)
#pi.set_PWM_frequency(13,800)
#pi.set_PWM_frequency(19,800)


#pi.set_mode(5, pigpio.OUTPUT)
#pi.set_mode(6, pigpio.OUTPUT)
#pi.set_mode(13, pigpio.OUTPUT)
#pi.set_mode(19, pigpio.OUTPUT)

rospy.spin()
