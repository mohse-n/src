#include <ros/ros.h>
#include <iostream>
#include "control_sys/ImuData.h"

class Node
{
public:
	Node(): SEq_1(0), SEq_2(0), SEq_3(0), SEq_4(1), gyroMeasError(3.14159265358979 * (1.0 / 180)), beta(0.1), delta_t(0.005), t_prev(ros::Time::now())
{
	pub = node.advertise<control_sys::ImuData>("imu_out", 1);
	sub = node.subscribe("imu_data", 1, &Node::callback, this);
	out.header.frame_id = "imu";
}
	// Madgwick filter code
	// Source : An efficient orientation filter for inertial and inertial/magnetic sensor array, Appendix A
	void callback(const control_sys::ImuData &msg)
{

	halfSEq_1 = 0.5 * SEq_1;
	halfSEq_2 = 0.5 * SEq_2;
	halfSEq_3 = 0.5 * SEq_3;
	halfSEq_4 = 0.5 * SEq_4;
	twoSEq_1 = 2 * SEq_1;
	twoSEq_2 = 2 * SEq_2;
	twoSEq_3 = 2 * SEq_3;

	norm = invSqrt(msg.quat.x * msg.quat.x + msg.quat.y * msg.quat.y + msg.quat.z * msg.quat.z);
	
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - msg.quat.x * norm;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - msg.quat.y * norm;
	f_3 = 1 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - msg.quat.z * norm;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2 * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2 * J_14or21; // negated in matrix multiplication
	J_33 = 2 * J_11or24;

	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

	// Normalise the gradient
	norm = invSqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + 		SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 *= norm;
	SEqHatDot_2 *= norm;
	SEqHatDot_3 *= norm;
	SEqHatDot_4 *= norm;
	
	w_x = msg.vel.x; w_y = msg.vel.y; w_z = msg.vel.z;

	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative
	delta_t = (ros::Time::now() - t_prev).toSec();
	t_prev = ros::Time::now();
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * delta_t;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * delta_t;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * delta_t;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * delta_t;

	// Normalise quaternion
	norm = invSqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);


	q1 = SEq_1*norm;
	q2 = SEq_2*norm;
	q3 = SEq_3*norm;
	q4 = SEq_4*norm;
	
	// Convert quaternions to euler angles
	// Possible source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	// Roll
	out.quat.x = std::atan((2*q1*q2 + 2*q3*q4)/(1-2*q2*q2 - 2*q3*q3)) * 57.295779513 ;
	// Pitch
	out.quat.y = std::asin(2*q1*q3 - 2*q4*q2) * 57.295779513;
	// Yaw
	out.quat.z = std::atan((2*q1*q4 + 2*q2*q3)/(1-2*q3*q3 - 2*q4*q4)) * 57.295779513;

	out.vel.x = msg.vel.x;
	out.vel.y = msg.vel.y;
   	out.vel.z = msg.vel.z;

	out.header.stamp = msg.header.stamp;
	
	pub.publish(out);
}

private:
	ros::NodeHandle node;
	ros::Publisher pub;
	ros::Subscriber sub;

	control_sys::ImuData out;

	const float gyroMeasError;
	const float beta;
    float delta_t;

	float SEq_1, SEq_2, SEq_3, SEq_4;

	float norm; // Vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derivative from gyroscopes elements
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;

	float halfSEq_1;
	float halfSEq_2;
	float halfSEq_3;
	float halfSEq_4;
	float twoSEq_1;
	float twoSEq_2;
	float twoSEq_3;
	float q1, q2, q3, q4;

	float w_x, w_y, w_z; // Gyro output
	
	float invSqrt(float );

	ros::Time t_prev;
	

};


// This function is taken from https://github.com/ccny-ros-pkg/imu_tools/blob/244f653063bf148be5a517f1bd5a4b6f92d31391/imu_filter_madgwick/src/imu_filter.cpp
float Node::invSqrt(float x)
{
	float xhalf = 0.5f * x;
	union
{
	float x;
	int i;
} u;
	u.x = x;
	u.i = 0x5f3759df - (u.i >> 1);
  /* The next line can be repeated any number of times to increase accuracy */
	u.x = u.x * (1.5f - xhalf * u.x * u.x);
	u.x = u.x * (1.5f - xhalf * u.x * u.x);
	return u.x;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "filter");

	Node filter;

	ros::spin();

}









