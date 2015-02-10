#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"test_commander");
	ros::NodeHandle nh;
	ROS_INFO("argc %d", argc);
	double vel = 0.5;
	double t = 30;
	if(argc >= 2) {
		vel = atof(*(argv+1));
		ROS_INFO("argv1 %f",vel);
	}
	if(argc >= 3) {
		t = atof(*(argv+2));
		ROS_INFO("argv2 %f",t);
	}
	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	ros::Rate sleep_timer(40);

	geometry_msgs::Twist twist_cmd;

	twist_cmd.linear.x = 0.0;
	twist_cmd.linear.y = 0.0;
	twist_cmd.linear.z = 0.0;
	twist_cmd.angular.x = 0.0;
	twist_cmd.angular.y = 0.0;
	twist_cmd.angular.z = 0.0;

	ROS_INFO("count-down");
	for (int j=3;j>0 && ros::ok();j--) {
	    ROS_INFO("launch in %d",j);
	    for (int i = 0; i<100;i++)
	        sleep_timer.sleep();
	}
	ROS_INFO("we have takeoff");

	int niters = (int) (40*t);

	twist_cmd.linear.x = vel;

	for (int i=0;i<niters && ros::ok();i++) {
	    cmd_publisher.publish(twist_cmd);
	    sleep_timer.sleep();
	}

	ROS_INFO("loop completed");
	for (int i = 0; i < 5 && ros::ok(); i++)
	{
		ROS_INFO("stopping");
		twist_cmd.linear.x = 0.0;
		twist_cmd.angular.z = 0;
		cmd_publisher.publish(twist_cmd);
	}

	return 0;
} 