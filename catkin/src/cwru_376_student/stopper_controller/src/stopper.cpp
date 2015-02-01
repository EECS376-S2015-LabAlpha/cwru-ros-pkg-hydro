#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist last_cmd;
std::string estop_status;
std::string lidar_status;
ros::Publisher cmd_pub;
double last_vel;
double max_decel = 0.1;

void estopCB(const std_msgs::Bool::ConstPtr& estop) {
	if (estop->data == true) { estop_status = "motors_ENABLED"; }
	else { estop_status = "motors_DISABLED"; }
}
void commandCB(const geometry_msgs::Twist& cmd) {
	if (strcmp("motors_DISABLED", estop_status.c_str()) == 0) {
		//ESTOP engaged
		last_cmd.linear.x = 0.0;
		last_cmd.angular.z = 0.0;
	}
	else if(strcmp("STOP",lidar_status.c_str())) {
		last_cmd.linear.x = std::max(0.0,last_vel-max_decel);
		last_cmd.angular.z = 0.0;
	}
	else { //pass through commands
		last_cmd.linear.x = cmd.linear.x;
		last_cmd.angular.z = cmd.angular.z;
	}
	cmd_pub.publish(last_cmd);

}
void lidarStopCB(const std_msgs::String& msg) {
	if (strcmp(msg.data.c_str(), "clear") == 0) { lidar_status = "CLEAR"; }
	else { lidar_status = "STOP"; }
}
void odomCB(const nav_msgs::Odometry& odom) {
	last_vel = odom.twist.twist.linear.x;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "stopper_controller");
	ros::NodeHandle n;
	cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

	ros::Subscriber estop_sub = n.subscribe("motors_enabled",1,estopCB);
	ros::Subscriber cmd_sub = n.subscribe("request_vel",1,commandCB);
	ros::Subscriber lidar_sub = n.subscribe("lidar_str",1,lidarStopCB);

	ros::spin();
}