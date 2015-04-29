// IM_driver.cpp
// William Baskin based on:
// Wyatt Newman, based on ROS tutorial 4.2 on Interactive Markers

#include <ros/ros.h>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

    
    
    
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
}

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
    geometry_msgs::Pose pose;
    pose.orientation = convertPlanarPhi2Quaternion(phi);
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;  
    return pose;
}

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w);
    return phi;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "IM_driver"); 

    interactive_markers::InteractiveMarkerServer server("driver_marker");
    
    visualization_msgs::InteractiveMarker int_marker;
    
    int_marker.header.frame_id = "/odom";
    int_marker.name = "des_robot_pose";
    int_marker.description = "Interactive Marker for Driving";

    geometry_msgs::Point temp_point_start;

    temp_point_start.x = 0.0; 
    temp_point_start.y = 0.0;
    temp_point_start.z = 0.0;

    
  /**/
    // create an arrow marker; do this 3 times to create a triad (frame)
    visualization_msgs::Marker arrow_marker_x; //this one for the x axis
    geometry_msgs::Point temp_point;

    arrow_marker_x.type = visualization_msgs::Marker::ARROW; //CUBE; //ROS example was a CUBE; changed to ARROW
    // specify/push-in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_x.points.push_back(temp_point);
    // Specify and push in the end point for the arrow 
    temp_point = temp_point_start;
    temp_point.x = 0.2; // arrow is this long in x direction
    temp_point.y = 0.0;
    temp_point.z = 0.0;
    arrow_marker_x.points.push_back(temp_point);

    // make the arrow very thin
    arrow_marker_x.scale.x = 0.01;
    arrow_marker_x.scale.y = 0.01;
    arrow_marker_x.scale.z = 0.01;

    arrow_marker_x.color.r = 1.0; // red, for the x axis
    arrow_marker_x.color.g = 0.0;
    arrow_marker_x.color.b = 0.0;
    arrow_marker_x.color.a = 1.0;

    // do this again for the y axis:
    visualization_msgs::Marker arrow_marker_y;
    arrow_marker_y.type = visualization_msgs::Marker::ARROW; 
    // Push in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_y.points.push_back(temp_point);
    // Push in the end point for the arrow 
    temp_point.x = 0.0;
    temp_point.y = 0.2; // points in the y direction
    temp_point.z = 0.0;
    arrow_marker_y.points.push_back(temp_point);

    arrow_marker_y.scale.x = 0.01;
    arrow_marker_y.scale.y = 0.01;
    arrow_marker_y.scale.z = 0.01;

    arrow_marker_y.color.r = 0.0;
    arrow_marker_y.color.g = 1.0; // color it green, for y axis
    arrow_marker_y.color.b = 0.0;
    arrow_marker_y.color.a = 1.0;

/**/
    // create a control that contains the markers
    visualization_msgs::InteractiveMarkerControl IM_control;
    IM_control.always_visible = true;
    //IM_control.markers.push_back(sphere_marker);
    
    IM_control.markers.push_back(arrow_marker_x);
    IM_control.markers.push_back(arrow_marker_y);
    
    // add the control to the interactive marker
    int_marker.controls.push_back(IM_control);

    // create a control that will move the marker
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl translate_control_x;
    translate_control_x.name = "move_x";
    translate_control_x.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    /** Create the Y-Axis Control*/
    visualization_msgs::InteractiveMarkerControl translate_control_y;
    translate_control_y.name = "move_y";
    translate_control_y.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    translate_control_y.orientation.x = 0; //point this in the y direction
    translate_control_y.orientation.y = 0;
    translate_control_y.orientation.z = 1;
    translate_control_y.orientation.w = 1;

    // add x-rotation control
  /**/
    // add y-rotation control
    visualization_msgs::InteractiveMarkerControl roty_control;
    roty_control.always_visible = true;
    roty_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    roty_control.orientation.x = 0;
    roty_control.orientation.y = 1;
    roty_control.orientation.z = 0;
    roty_control.orientation.w = 1;
    roty_control.name = "rot_z";
/**/
    // add the controls to the interactive marker
    int_marker.controls.push_back(translate_control_x);    
    int_marker.controls.push_back(translate_control_y);
    int_marker.controls.push_back(roty_control);
    
    /** Scale Down: this makes all of the arrows/disks for the user controls smaller than the default size */
    int_marker.scale = 0.2;
    
    //let's pre-position the marker, else it will show up at the frame origin by default
    int_marker.pose.position.x = temp_point_start.x;
    int_marker.pose.position.y = temp_point_start.y;
    int_marker.pose.position.z = temp_point_start.z;
    
    ros::NodeHandle nh; 
    ros::ServiceClient client = nh.serviceClient<cwru_srv::path_service_message>("appendPathService");

    cwru_srv::path_service_message path_message;
    geometry_msgs::PoseStamped vertex;  // this contains a header and a pose; the pose contains a point and a quaternion
    double x,y,phi;
    vertex.header.stamp = ros::Time::now();


    geometry_msgs::Quaternion quaternion2;
    quaternion2.x = int_marker.pose.orientation.x;
    quaternion2.y = int_marker.pose.orientation.y;
    quaternion2.z = int_marker.pose.orientation.z;
    quaternion2.w = int_marker.pose.orientation.w;
    x=int_marker.pose.position.x;
    y=int_marker.pose.position.y;
    phi=convertPlanarQuat2Phi(quaternion2);
    ROS_INFO("vertex: x,y,phi = %f, %f %f",x,y,phi);
    vertex.pose = xyPhi2Pose(x,y,phi); //x,y,phi  
    path_message.request.path.poses.push_back(vertex);


    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, &processFeedback);

    // 'commit' changes and send to all clients
    server.applyChanges();
    


    // start the ROS main loop
    ROS_INFO("going into spin...");
    ros::spin();
}


