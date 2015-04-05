//example path sender starting pen
// wsn, March 2015
// test node compatible with example_des_state_generator.cpp;
// transmits a hard-coded path to desired-state generator node via service "appendPathService"
// the message must contain a nav_msgs/Path object
// hard-coded nodes compatible with the map: startingPenMap in cwru_urdf
// poses (x,y) = (0,0), (5,0), (5,5), (-3,5), (-3,0), (0,0)

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

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

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

int main(int argc, char **argv) {
    double dt=0.01;
    ros::init(argc, argv, "test_path_sender"); // name of this node 
    ros::NodeHandle nh; 
    ros::ServiceClient client = nh.serviceClient<cwru_srv::path_service_message>("appendPathService");

    cwru_srv::path_service_message path_message;
    geometry_msgs::PoseStamped vertex;  // this contains a header and a pose; the pose contains a point and a quaternion
    double x,y,phi;

    vertex.header.stamp = ros::Time::now(); // look up the time and put it in the header; use same time stamp for all vertices in this path
    vertex.header.frame_id = "map"; // specify this, so tf will know how to transform it
    
    // fill in the interesting data: (x,y) and phi = location and heading
    //vertex 1:
    geometry_msgs::Quaternion quaternion1;
    quaternion1.x = 0.0;
    quaternion1.y = 0.0;
    quaternion1.z = -0.9092;
    quaternion1.w = 0.4162;
    x=8.682;
    y=15.881;
    phi=convertPlanarQuat2Phi(quaternion1);
    ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi
    path_message.request.path.poses.push_back(vertex);

    //vertex 2:
    geometry_msgs::Quaternion quaternion2;
    quaternion2.x = 0.0;
    quaternion2.y = 0.0;
    quaternion2.z = 0.9305;
    quaternion2.w = 0.3663;
    x=5.099;
    y=12.208;
    phi=convertPlanarQuat2Phi(quaternion2);
    ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi  
    path_message.request.path.poses.push_back(vertex);
    
    //vertex 3:
    geometry_msgs::Quaternion quaternion3;
    quaternion3.x = 0.0;
    quaternion3.y = 0.0;
    quaternion3.z = 0.3725;
    quaternion3.w = 0.9280;
    x=-3.211;
    y=20.7907;
    phi=convertPlanarQuat2Phi(quaternion3);
    ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi  
    path_message.request.path.poses.push_back(vertex);

        //vertex 4:
    geometry_msgs::Quaternion quaternion4;
    quaternion4.x = 0.0;
    quaternion4.y = 0.0;
    quaternion4.z = -0.369;
    quaternion4.w = 0.9302;
    x=3.4058;
    y=26.9174;
    phi=convertPlanarQuat2Phi(quaternion4);
    ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi  
    path_message.request.path.poses.push_back(vertex);
    

    
    //transmit this path message:
    if (client.call(path_message)) {
        ROS_INFO("got ack from server");
        } else {
            ROS_ERROR("Failed to call service lookup_by_name");
            return 1;
        }
    return 0;
}
