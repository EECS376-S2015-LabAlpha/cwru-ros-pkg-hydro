//example path sender
// wsn, Feb 2015
// test node compatible with example_des_state_generator.cpp;
// transmits a hard-coded path to desired-state generator node via service "appendPathService"
// the message must contain a nav_msgs/Path object

#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

geometry_msgs::Pose IM_pose;
ros::ServiceClient client;

//utility to convert from heading to quaternion, planar motion
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi/2.0);
    quaternion.w = cos(phi/2.0);
    return(quaternion);
}

//utility to fill a Pose object from planar x,y,phi info
geometry_msgs::Pose xyPhi2Pose(double x, double y, double phi) {
    geometry_msgs::Pose pose; // a pose object to populate
    pose.orientation = convertPlanarPhi2Quaternion(phi); // convert from heading to corresponding quaternion
    pose.position.x = x; // keep the robot on the ground!
    pose.position.y = y; // keep the robot on the ground!    
    pose.position.z = 0.0; // keep the robot on the ground!  
    return pose;
}

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    //Make sure that the returned angle is within +- 2 * pi
    /*while (phi < -6.2831853) {
        phi += 6.2831853;
    }
    while (phi > 6.2831853) {
        phi -= 6.2831853;
    }*/
    return phi;
}

//use this service to set processing modes interactively
bool modeService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("mode select service callback activated");
    response.resp = true; // boring, but valid response info
    //g_pcl_process_mode = request.req;
    //g_trigger = true; //signal that we received a request; trigger a response
    ROS_INFO("service callback!");


    cwru_srv::path_service_message path_message;
    geometry_msgs::PoseStamped vertex;  // this contains a header and a pose; the pose contains a point and a quaternion
    double x,y,phi;
    vertex.header.stamp = ros::Time::now();


    geometry_msgs::Quaternion quaternion1;
    quaternion1.x = IM_pose.orientation.x;
    quaternion1.y = IM_pose.orientation.y;
    quaternion1.z = IM_pose.orientation.z;
    quaternion1.w = IM_pose.orientation.w;
    x=IM_pose.position.x;
    y=IM_pose.position.y;
    phi=convertPlanarQuat2Phi(quaternion1);
    ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);
    if (client.call(path_message)) {
        ROS_INFO("got ack from server");
        } else {
            ROS_ERROR("Failed to call service lookup_by_name");

        }

}




void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
    IM_pose.position.x = feedback->pose.position.x;
    IM_pose.position.y = feedback->pose.position.y;
    IM_pose.orientation.x = feedback->pose.orientation.x;
    IM_pose.orientation.y = feedback->pose.orientation.y;
    IM_pose.orientation.z = feedback->pose.orientation.z;
    IM_pose.orientation.w = feedback->pose.orientation.w;

}

int main(int argc, char **argv) {
    double dt=0.01;
    ros::init(argc, argv, "test_path_sender"); // name of this node 
    ros::NodeHandle nh; 
    client = nh.serviceClient<cwru_srv::path_service_message>("appendPathService");


    ros::ServiceServer service = nh.advertiseService("drive_mode", modeService);
    //ros::Subscriber sub = nh.subscribe("/robot0/odom", 1, odomCallback);
    //ros::Rate rtimer(1 / DT); // frequency corresponding to chosen sample period 

    ros::Subscriber IM_subscriber_ = nh.subscribe("/driver_marker/feedback", 1, processFeedback);
    //cwru_srv::path_service_message path_message;
    //geometry_msgs::PoseStamped vertex;  // this contains a header and a pose; the pose contains a point and a quaternion
    //double x,y,phi;
    //ROS_WARN(g_pcl_process_mode);
    //vertex.header.stamp = ros::Time::now(); // look up the time and put it in the header; use same time stamp for all vertices in this path
    
    // fill in the interesting data: (x,y) and phi = location and heading
    //vertex 1:
    ros::spin();
}
