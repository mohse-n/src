#include <ros/ros.h>
#include <iostream>
#include "control_sys/ImuData.h"
#include <dynamic_reconfigure/server.h>
#include <control_sys/mixer_paramConfig.h>

class Node
{

public:
	Node() : u_lim(254), l_lim(125), firstMsg(1), secondMsg(0), motor_on(0), motor_init(1), reset(1), start_sig(125), off_sig(100)
{
		// Published and subscribed topics
		pub = node.advertise<control_sys::ImuData>("motor_cmd", 1);		
		sub = node.subscribe("cont1_out", 1, &Node::callback, this);

		// The dynamic parameter server's callback function is called callback_server
		f = boost::bind(&Node::callback_server, this, _1, _2);
		server.setCallback(f);
}

	void callback(const control_sys::ImuData &msg)
{
		// getCached requests an update only when the value changes and not on every callback -> better performance
		ok = ros::param::getCached("/angle_cont/reset", reset);

		// If motor_init or reset is enabled, we don't want to take-off yet. So don't bother with the P controllers. (Notice the "return"s)
		if (motor_init || reset)
{
			/* If this is the first message, send the startup signal and then wait to make sure that the motors have started.
			   Note that if we don't halt after sending the signal, the motors will "ignore" it.
			   I multiply the startup signal by motor_on. If we haven't set the motor_on parameter, we don't want the motors 
			   starting when they receive the first message! */
			if (firstMsg)
{
				out.quat.x = motor_on * start_sig;
				out.quat.z = motor_on * start_sig;
				out.quat.y = motor_on * start_sig; 
				out.quat.w = motor_on * start_sig;
				pub.publish(out);
				ros::Duration(0.5).sleep();
				firstMsg = 0;
				secondMsg = 1;
				return;
}			if (secondMsg)
{				
				out.quat.x = motor_on * off_sig;
				out.quat.z = motor_on * off_sig;
				out.quat.y = motor_on * off_sig;	
				out.quat.w = motor_on * off_sig;
				pub.publish(out);
				ros::Duration(1).sleep();
				secondMsg = 0;
				return;
}	
			// If we don't want to fly yet AND it's not the first message, motors should spin at the lowest possible speed
			out.quat.x = motor_on * l_lim; out.quat.z = motor_on * l_lim; out.quat.y = motor_on * l_lim; out.quat.w = motor_on * l_lim;

			pub.publish(out);
			/* We might want to restart (off-on) the motors. We need send the startup signal again.
			   This way if the motors are off we can satisfy the firstMsg condition again. */
			firstMsg = !motor_on;
			return;	
}	
		
		throttle_cmd = throttle_bias + msg.quat.w * z_mixer; 
		// Contribution of roll, pitch and yaw to motor commands
		roll_cont = (msg.quat.x - msg.vel.x) * roll_p * roll_mixer;
		pitch_cont = (msg.quat.y - msg.vel.y) * pitch_p * pitch_mixer;
		yaw_cont = (msg.quat.z - msg.vel.z) * yaw_p * yaw_mixer;
		std::cout << msg.quat.w * z_mixer << "   " << roll_cont << "   " << pitch_cont << "   "  << 		yaw_cont << '\n';
	
		// Saturate the controller output.
		 out.quat.x = motor_on * sat( throttle_cmd + pitch_cont + yaw_cont );
		 out.quat.z = motor_on * sat( throttle_cmd - pitch_cont + yaw_cont );
	
		 out.quat.y = motor_on * sat( throttle_cmd + roll_cont - yaw_cont );
		 out.quat.w = motor_on * sat( throttle_cmd - roll_cont - yaw_cont );	

		pub.publish(out); 

}
	// Get the dynamic parameters from the server
	void callback_server(control_sys::mixer_paramConfig &config, uint32_t level)
{
		roll_p = config.roll_p; pitch_p = config.pitch_p; yaw_p = config.yaw_p;	

		roll_mixer = config.roll_mixer; pitch_mixer = config.pitch_mixer; yaw_mixer = config.yaw_mixer; 		z_mixer = config.z_mixer;

		throttle_bias = config.throttle_bias;

		motor_on = config.motor_on; motor_init = config.motor_init;
	
}
	// Saturation function
	inline int sat(const float &signal) const
{
		return (signal > u_lim) ? u_lim : (signal < l_lim) ? l_lim : signal;
} 

private:

	ros::NodeHandle node;
	ros::Publisher pub;
	ros::Subscriber sub;

	control_sys::ImuData out;
	float roll_p, pitch_p, yaw_p;
	float roll_mixer, pitch_mixer, yaw_mixer, z_mixer, throttle_bias;
	float roll_cont, pitch_cont, yaw_cont;
	int throttle_cmd;
	bool motor_on, motor_init, reset, firstMsg, secondMsg, ok;
	const int u_lim, l_lim;
	// start_sig: Value given to the motor for startup
	const float start_sig, off_sig;

	dynamic_reconfigure::Server<control_sys::mixer_paramConfig> server;
	dynamic_reconfigure::Server<control_sys::mixer_paramConfig>::CallbackType f;   
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "mixer");

	Node mixer;

	ros::spin();

}



