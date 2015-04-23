// simple_steering.h header file //
// wcb38; 1000-03-06-2015

#ifndef LABALPHA_STEERING_SIMPLE_H_
#define LABALPHA_STEERING_SIMPLE_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//Eigen is useful for linear algebra
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

const double UPDATE_RATE = 50.0; // choose the desired-state publication update rate
const double K_PHI= 10.0; // control gains for steering
const double K_DISP = 3.0;
const double K_TRIP_DIST = 1.0;
// dynamic limitations:  these apply to the steering controller; they may be larger than the limits on des state generation
const double MAX_SPEED = 1.0; // m/sec; adjust this
const double MAX_OMEGA = 1.0; //1.0; // rad/sec; adjust this
const double MAX_ALPHA = 0.1;


// define a class, including a constructor, member variables and member functions
class SteeringController
{
public:
    SteeringController(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    void my_clever_steering_algorithm(); // here is the heart of it...use odom state and desired state to compute twist command, and publish it
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);   
    double min_dang(double dang);  
    double sat(double x);
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    ros::Subscriber cur_state_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber des_state_subscriber_;
    
    ros::Publisher cmd_publisher_; // = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Publisher cmd_publisher2_; // = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1);
    ros::Publisher steering_errs_publisher_;
    
    geometry_msgs::Twist twist_cmd_;
    geometry_msgs::TwistStamped twist_cmd2_;    
    double current_speed_des_;
    double current_omega_des_;
    

    //state values from odometry; these will get filled in by odom callback
    nav_msgs::Odometry current_odom_; // fill in these objects from callbacks
    geometry_msgs::Pose odom_pose_;    
    double odom_vel_;
    double odom_omega_;
    double odom_x_;
    double odom_y_;
    double odom_phi_;
    geometry_msgs::Quaternion odom_quat_; 
    Eigen::Vector2d odom_xy_vec_;
    
    //state values from desired state; these will get filled in by desStateCallback
    nav_msgs::Odometry des_state_; 
    geometry_msgs::Pose des_state_pose_;    
    double des_state_vel_;
    double des_state_omega_;
    double des_state_x_;
    double des_state_y_;
    double des_state_phi_;
    geometry_msgs::Quaternion des_state_quat_;  
    Eigen::Vector2d des_xy_vec_;    
    
    // message to hold/publish steering performance data
    std_msgs::Float32MultiArray steering_errs_;
        
    // member methods as well:
    void initializeSubscribers();
    void initializePublishers();
    
    void currentStateCB(const nav_msgs::Odometry& odom_rcvd);
    void desStateCallback(const nav_msgs::Odometry& des_state_rcvd);    
        
};

#endif