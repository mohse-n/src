#include <ros/ros.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "control_sys/Float64Stamped.h"
#include "control_sys/ImuData.h"
#include <dynamic_reconfigure/server.h>
#include <control_sys/angle_paramConfig.h>

using namespace message_filters;
// Ultrasonic sensor data
typedef control_sys::Float64Stamped usMsg;
// IMU sensor data
typedef control_sys::ImuData imuMsg;

class Node
{
	
public:

	Node() : roll_intg(0), pitch_intg(0), yaw_intg(0), z_intg(0), roll_cmd(0), pitch_cmd(0), intg_lim(10), cmd_deg(0.174533), t_prev(ros::Time::now()), c(10)
{
		// Published and subscribed topics
		pub = node.advertise<imuMsg>("cont1_out", 1);	
		sub_imu.subscribe(node, "imu_out", 1);
		sub_us.subscribe(node, "us_out", 1);
		
		// Sync the IMU and sonar messages 
		sync.reset(new Sync(MySyncPolicy(5), sub_imu, sub_us)); 
		// The synchronizer's callback function is called callback
		sync -> registerCallback(boost::bind(&Node::callback, this, _1, _2));
		// The dynamic parameter server's callback function is called callback_server
		// bind syntax: newCallable = bind(callable, arg_list). 
		// _1, _2 : the first and second arguments of "f" are also args of "callback_server"
		f = boost::bind(&Node::callback_server, this, _1, _2);
		server.setCallback(f);

	 
}
	// (Synchronized) subscriber callback. "imu_out" & "us_out" are references to pointers e.g. float *(&pr) = p; 
	// Because they are pointers, I will be derefrencing them to access their members e.g. (*imu_out).quat.x
	void callback(const control_sys::ImuDataConstPtr &imu_out, const control_sys::Float64StampedConstPtr &us_out)
{
		dt = (ros::Time::now() - t_prev).toSec();
		t_prev = ros::Time::now();
	
		// Update rate of the angle and altitude controllers is c = 10 times slower than that of the rate controllers
		if (c == 10)
{
			alt_fb = (*us_out).data;
		
			// The altidude feedback can be 10 centimeters below or above our command, and we would not consider it as an error. (10 cm tolerance)
			// We don't want motors changing speed for minute changes in sonar readings!
			if (alt_fb > (alt_cmd - 10) && alt_fb < (alt_cmd + 10))
				alt_fb = alt_cmd;

			// Compute the integral terms
			roll_intg = sat( roll_intg + roll_i * (roll_cmd - (*imu_out).quat.x) * dt );
			pitch_intg = sat( pitch_intg + pitch_i * (pitch_cmd - (*imu_out).quat.y) * dt ); 
			yaw_intg = sat( yaw_intg + yaw_i * (yaw_cmd - (*imu_out).quat.z) * dt );  
			z_intg = sat( z_intg + z_i * (alt_cmd - alt_fb) * dt );
	
			// Reset the integral terms. We don't want them to buildup while we're preparing to fly!
			if (reset)
{
				roll_intg = 0; pitch_intg = 0; yaw_intg = 0; z_intg = 0;
	
}
			out.quat.x = roll_p * (roll_cmd -(*imu_out).quat.x) + roll_intg;
			out.quat.y = pitch_p * (pitch_cmd -(*imu_out).quat.y) + pitch_intg;
			out.quat.z = yaw_p * (yaw_cmd -(*imu_out).quat.z) + yaw_intg;
			out.quat.w = z_p * (alt_cmd - alt_fb) + z_intg;
		
			std::cout << roll_intg << "   " << pitch_intg << "   " << yaw_intg << "   " << z_intg << '\n';
			c = 0;
}
		++c;
		
		// Publish the rates with the normal rate
		out.vel.x = (*imu_out).vel.x; out.vel.y = (*imu_out).vel.y; out.vel.z = (*imu_out).vel.z;

		pub.publish(out);
		
        
}  
	// Get the dynamic parameters from the server
	void callback_server(control_sys::angle_paramConfig &config, uint32_t level)
{

		roll_p = config.roll_p; roll_i = config.roll_i;
		pitch_p = config.pitch_p; pitch_i = config.pitch_i;
		yaw_p = config.yaw_p; yaw_i = config.yaw_i;
		z_p = config.z_p; z_i = config.z_i;
	
		// Altitude command is given in centimeters
		alt_cmd = config.altitude_cmd;
		roll_cmd = cmd_deg * config.roll_cmd;
		pitch_cmd = cmd_deg * config.pitch_cmd;
		yaw_cmd = config.yaw_cmd;

		reset = config.reset;
	
}
	// Saturate an integral term
	inline float sat(const float &intg)
{
		return (intg > intg_lim) ? intg_lim : (intg < -intg_lim) ? -intg_lim : intg;
} 

private:

	ros::NodeHandle node;
	ros::Publisher pub;
	
	imuMsg out;
	float roll_p, roll_i, pitch_p, pitch_i, yaw_p, yaw_i, z_p, z_i;
	float roll_cmd, pitch_cmd, yaw_cmd, alt_cmd;
	float roll_intg , pitch_intg , yaw_intg, z_intg;
	const float intg_lim;
	// Default cmd_deg = 2 deg
	const float cmd_deg;
	ros::Time t_prev;
	float dt;
	bool reset;	
	int c;	
	float alt_fb;

	message_filters::Subscriber<imuMsg> sub_imu;
	message_filters::Subscriber<usMsg> sub_us;

	typedef sync_policies::ApproximateTime<imuMsg, usMsg> MySyncPolicy;
	typedef Synchronizer<MySyncPolicy> Sync;
	boost::shared_ptr<Sync> sync;

	// Declare the server and the callback function
	dynamic_reconfigure::Server<control_sys::angle_paramConfig> server;
	dynamic_reconfigure::Server<control_sys::angle_paramConfig>::CallbackType f;  
	

	
};





int main(int argc, char **argv)
{

	ros::init(argc, argv, "angle_cont");
	Node angle_cont;

	ros::spin();
	
}
