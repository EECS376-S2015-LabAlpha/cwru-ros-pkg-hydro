#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist last_cmd;
std::string estop_status = "motors_ENABLED";
std::string lidar_status = "CLEAR";
std::string input_status = "CLEAR";
std_msgs::String last_cli;
ros::Publisher cmd_pub;
double last_vel;
double max_decel = 0.1;

void estopCB(const std_msgs::Bool::ConstPtr& estop) {
	if (estop->data == true) { estop_status = "motors_ENABLED"; }
	else { estop_status = "motors_DISABLED"; }
}
void commandCB(const geometry_msgs::Twist& cmd) {
	ROS_INFO("input stop | %d",(strcmp(last_cli.data.c_str(), "CLEAR")));
	ROS_INFO("input stat | %s", input_status.c_str());
	if (strcmp("motors_DISABLED", estop_status.c_str()) == 0) {
		ROS_INFO("estop stop");
		//ESTOP engaged
		last_cmd.linear.x = 0.0;
		last_cmd.angular.z = 0.0;
	}
	else if(strcmp("STOP",lidar_status.c_str())==0) {
		ROS_INFO("lidar stop");
		//Lidar stop requested
		last_cmd.linear.x = std::max(0.0,last_vel-max_decel);
		last_cmd.angular.z = 0.0;
	}
	else if(strcmp("STOP", input_status.c_str())==0) {
		ROS_INFO("input stop");
		// CLI stop requested
		last_cmd.linear.x = std::max(0.0,last_vel-max_decel);
		last_cmd.angular.z = 0.0;
	}
	else { //pass through commands, no stop necessary
		ROS_INFO("pass thru");
		last_cmd.linear.x = cmd.linear.x;
		if (last_cmd.linear.x > .01 && last_cmd.linear.x < .11) { last_cmd.linear.x = .11; }
		last_cmd.angular.z = cmd.angular.z;
	}
	cmd_pub.publish(last_cmd);
	ROS_INFO("last_cmd %f", last_cmd.linear.x);

}
void lidarStopCB(const std_msgs::Bool& msg) {
	if (!msg.data) { lidar_status = "CLEAR"; }
	else { lidar_status = "STOP"; }
}
void typeStopCB(const std_msgs::String& msg) {
	if (strcmp(msg.data.c_str(), "CLEAR") == 0) { input_status = "CLEAR"; }
	else { input_status = "STOP"; }
	//input_status = "CLEAR";
	last_cli = msg;
}
void odomCB(const nav_msgs::Odometry& odom) {
	last_vel = odom.twist.twist.linear.x;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "stopper_controller");
	ros::NodeHandle n;

	//input_status = "CLEAR";

	//publish on the cmd_velocity topic
	cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	//subscribe to the current state of the e-stop
	ros::Subscriber estop_sub = n.subscribe("/motors_enabled",1,estopCB);
	//subscribe to requested velocity
	ros::Subscriber cmd_sub = n.subscribe("/request_vel",1,commandCB);
	//subscribe to lidar stop output
	ros::Subscriber lidar_sub = n.subscribe("/lidar_alarm",1,lidarStopCB);
	//subscribe to control strings from command line
	ros::Subscriber cli_sub = n.subscribe("/cmd_str",1,typeStopCB); 
	//subscribe to odom for proper slowdown on CLI, Lidar stops
	ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCB); //need to change for jinx

	ros::spin();
}
