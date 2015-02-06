# stopper_controller

Takes input from lidar, estop, and the command line to stop the robot when either is active.
Else, it pipes the requested velocity to the cmd_vel channel.

Subscribe:
EStop 				| "motors_enabled"	| (std_msgs::Bool)
Request velocity 	| "request_vel" 	| (geometry_msgs::Twist)
Lidar stop/alarm	| "lidar_str" 		| (std_msgs::String)
CLI stop/alarm		| "cmd_str" 		| (std_msgs::String)
Odometry			| "jinx/odom" 		| (nav_msgs::Odometry)

Publish:
Command velocity	| "jinx/cmd_vel"	| (geometry_msgs::Twist)

## Example usage
rosrun stopper_controller stopper

## Running tests/demos
    
