#!/usr/bin/env python
import rospy
from control_sys.msg import Float64Stamped
import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)

TRIG = 23 
ECHO = 24

rospy.init_node('ultrasonic')
pub =rospy.Publisher('us_out', Float64Stamped, queue_size = 1)

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

GPIO.output(TRIG, False)
out = Float64Stamped()
rate = rospy.Rate(200)

# Let's assume the very first distance is 10 cm
# Therefore pulse_duration = 10/17150 = 0.0005831
pulse_end = 0.0005831
pulse_start = 0

while not rospy.is_shutdown():

	GPIO.output(TRIG, True)
	rospy.sleep(0.00001)
	GPIO.output(TRIG, False)

	c = 1000

	while GPIO.input(ECHO) == 0:
		#######			Better precision with time.time()       ############
 		#pulse_start = rospy.Time.now()
		pulse_start = time.time()
		# Timeout
		c -= 1
		if c == 0:
			break

	c = 1000
	
	while GPIO.input(ECHO) == 1:
		#pulse_end = rospy.Time.now()
		pulse_end = time.time()
		c -= 1
		if c == 0:
			break

		pulse_duration = pulse_end - pulse_start

	distance = pulse_duration * 17150

	# A method for removing outliers
	# Given an update rate of 100 hz, there is no way the quadrotor travels 3 cm in 
	# vertical direction in 0.01 sec.
	if not (distance > out.data + 3 or distance < out.data - 3):
		out.data = round(distance, 2)
	
	out.header.stamp = rospy.Time.now()
	pub.publish(out)	
	rate.sleep()

