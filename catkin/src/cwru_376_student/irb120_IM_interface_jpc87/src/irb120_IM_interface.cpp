// simple_marker_listener.cpp
// Wyatt Newman
// node that listens on topic "marker_listener" and prints pose received

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>
#include <irb120_kinematics.h>

//callback to subscribe to marker state
Eigen::Vector3d g_p; //where I want to go
Vectorq6x1 g_q_state; //where I be at
double g_x,g_y,g_z;
double frequency = 10.0;
//geometry_msgs::Quaternion g_quat; // global var for quaternion
Eigen::Quaterniond g_quat;
Eigen::Matrix3d g_R;
Eigen::Affine3d g_A_flange_desired;
bool g_trigger=false;
using namespace std;

void markerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    //ROS_INFO_STREAM(feedback->marker_name << " is now at "
      //      << feedback->pose.position.x << ", " << feedback->pose.position.y
        //    << ", " << feedback->pose.position.z);
    //copy to global vars:
    g_p[0] = feedback->pose.position.x;
    g_p[1] = feedback->pose.position.y;
    g_p[2] = feedback->pose.position.z;
    g_quat.x() = feedback->pose.orientation.x;
    g_quat.y() = feedback->pose.orientation.y;
    g_quat.z() = feedback->pose.orientation.z;
    g_quat.w() = feedback->pose.orientation.w;   
    g_R = g_quat.matrix();
}

void jointStateCB(
const sensor_msgs::JointStatePtr &js_msg) { //THIS IS NOT GETTING CALLED!!!! I DONT KNOW WHY... FIX!!!
    
    for (int i=0;i<6;i++) {
        g_q_state[i] = js_msg->position[i];
        //ROS_INFO("q_p_state is %f for %d",g_q_state[i],i);
    }
    //cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
    
}

//start or stop continous motion
bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("service callback activated");
    response.resp = true; //response
    
    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    g_trigger=true; //flag
    
    return true;
}

//Used to get appropriate time based on joint movements
double getTimeTraversalFromJoints(Vectorq6x1 initial_state, Vectorq6x1 end){
    double weight = 0;
    double duration = 0;
    for(int joints = 0; joints < 6; joints++){
        switch(joints){
            case 0:         //greatest importance.. go slow on big movements
                weight = 3;
                break;
            case 1:
                weight = 2;
                break;
            case 2:
                weight = .8;
                break;
            default:
                weight = .2; //we dont really care for the small guys in terms of subdivisions
        }
        
        duration +=  weight * std::abs(initial_state[joints] - end[joints]);
        
    }
    ROS_INFO("time is %f",duration);
    return duration;
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void stuff_trajectory( std::vector<Vectorq6x1> qvec_array, int number_of_vecs, trajectory_msgs::JointTrajectory &new_trajectory, double wait_time) {

    new_trajectory.points.clear();

    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");  

    new_trajectory.header.stamp = ros::Time::now();

    Vectorq6x1 qvec;
    double total_duration = 0;
    Vectorq6x1 initial_state = g_q_state; 
    for(int vec = 0;vec < number_of_vecs;vec++){ //Lets traverse over the vectors that came in
        qvec = qvec_array[vec];//Get the nth element
        double time_to_traverse = getTimeTraversalFromJoints(initial_state,qvec); //Get how much time this should take
        double dt = .2; //Lets split things up
        double points = time_to_traverse/dt; //take our calculated time it takes and subdivide into points

        for (int point=0;point<points;point++) { //Lets go thorugh all the points
            trajectory_msgs::JointTrajectoryPoint trajectory_point;
            trajectory_point.positions.clear(); 
            for (int ijnt=0;ijnt<6;ijnt++) { //In each point we have to update the individual joints
                trajectory_point.positions.push_back(initial_state[ijnt] + ( qvec[ijnt] - initial_state[ijnt]) * (point / points)); //for each dt add another point
            }  

            trajectory_point.time_from_start =  ros::Duration(dt * point + total_duration); //The time where each segment ends is the total so far plus the dt
            new_trajectory.points.push_back(trajectory_point); // append this point to trajectory
        }

        for (int ijnt=0;ijnt<6;ijnt++) {
            initial_state[ijnt] = qvec[ijnt];//Update the new current position after move is made
        } 
        
        total_duration = total_duration +  time_to_traverse + wait_time;//Update time for the next vector
    }
}

//Used to determine the best solution among many solutions
int findOptimalSolution (std::vector<Vectorq6x1> solutions, double count){
    int maxSize; //use this to determine the solutions to iterate through
    if(count > 50) //lets make sure we dont have infinity solutions.... 
    {
        ROS_INFO("Too many solutions... choosing from the first 50");
        maxSize = 50;
    }
    else maxSize = count; //lets use what came in

    double minweight = 0; //Use this and bestIndex to keep track of the best option
    double bestindex = 0;
    for(int sol = 0; sol < maxSize; sol++){
            double weight = 0;
            for(int joints = 0; joints < 6; joints++){
                weight = 0;
                switch(joints){
                    case 0:
                        weight = weight + std::abs(g_q_state[joints] - solutions[sol][joints]) * 10 + std::abs(solutions[sol][joints]) * 20; //Make it really pleasing to be in the up position
                        break;
                    case 1:
                        weight =  weight + std::abs(g_q_state[joints] - solutions[sol][joints]) * 7 + (solutions[sol][joints] + M_PI/2) * 10; //And upright for the second...
                        break;
                    case 2:
                        weight =  weight + std::abs(g_q_state[joints] - solutions[sol][joints]) * 4 - (solutions[sol][joints] - M_PI/2) * 10; //And third joints.
                        break;
                    case 3:
                        weight =  weight + std::abs(g_q_state[joints] - solutions[sol][joints]) * 2; //The rest isn't that crucial
                        break;
                    case 4:
                        weight =  weight + std::abs(g_q_state[joints] - solutions[sol][joints]) * 1;
                        break;
                    case 5:
                        weight =  weight + std::abs(g_q_state[joints] - solutions[sol][joints]) * 1;
                        break;
                }
            }
            //ROS_INFO("With weight %f and 1st joint at %f", weight, solutions[sol][0]); //Lets print out the important stuff
            if(weight < minweight){//Lets update the best choice
                minweight = weight;
                bestindex = sol;
            }
        }
    return bestindex;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_listener"); // this will be the node name;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    ROS_INFO("setting up subscribers ");
    ros::Subscriber sub_js = nh.subscribe("/joint_states",1,jointStateCB);
    ros::Subscriber sub_im = nh.subscribe("example_marker/feedback", 1, markerListenerCB);
    ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);   
    
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    std::vector<Vectorq6x1> q6dof_solns;
    std::vector<Vectorq6x1> q6dof_desired_poses(2); //Array of poses to feed in
    Vectorq6x1 qvec;
    Vectorq6x1 homevec;
    ros::Rate sleep_timer(frequency); //10Hz update rate    
    Irb120_fwd_solver irb120_fwd_solver; //instantiate forward and IK solvers
    Irb120_IK_solver ik_solver;
    Eigen::Vector3d n_urdf_wrt_DH,t_urdf_wrt_DH,b_urdf_wrt_DH;
    // in home pose, R_urdf = I
    //DH-defined tool-flange axes point as:
    // z = 1,0,0
    // x = 0,0,-1
    // y = 0,1,0
    // but URDF frame is R = I
    // so, x_urdf_wrt_DH = z_DH = [0;0;1]
    // y_urdf_wrt_DH = y_DH = [0;1;0]
    // z_urdf_wrt_DH = -x_DH = [-1; 0; 0]
    // so, express R_urdf_wrt_DH as:
    n_urdf_wrt_DH <<0,0,1;
    t_urdf_wrt_DH <<0,1,0;
    b_urdf_wrt_DH <<-1,0,0;
    Eigen::Matrix3d R_urdf_wrt_DH;
    R_urdf_wrt_DH.col(0) = n_urdf_wrt_DH;
    R_urdf_wrt_DH.col(1) = t_urdf_wrt_DH;
    R_urdf_wrt_DH.col(2) = b_urdf_wrt_DH;    

    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory

    
    homevec<<0,-M_PI/3,-M_PI/3,0,M_PI/3,0; //home state... deriably with 0, -pi/2 , pi/2 to start. This is used so that it can always get to the table from above
    Eigen::Affine3d A_flange_des_DH;
    
    //Eigen::Affine3d A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(qvec); //fwd_kin_solve
    
    int nsolns;
    while(ros::ok()) {
            ros::spinOnce();

            if (g_trigger) {
                // ooh!  excitement time!  got a new tool pose goal!
                g_trigger=false; // reset the trigger
                //is this point reachable?
                A_flange_des_DH = g_A_flange_desired;
                A_flange_des_DH.linear() = g_A_flange_desired.linear()*R_urdf_wrt_DH.transpose();
                //cout<<"R des DH: "<<endl;
                //cout<<A_flange_des_DH.linear()<<endl;
                nsolns = ik_solver.ik_solve(A_flange_des_DH);
                ROS_INFO("there are %d solutions",nsolns);

                if (nsolns>0) {      
                    ik_solver.get_solns(q6dof_solns);
                    qvec = q6dof_solns[findOptimalSolution(q6dof_solns, nsolns)]; // Choose the first solution wisely

                    q6dof_desired_poses[0] = qvec; //first go to my first point
                    q6dof_desired_poses[1] = homevec;  //then my own defined home
                                   
                    stuff_trajectory(q6dof_desired_poses,2,new_trajectory, 2); //2 poses feed in with a 2 sec delay in between each one
 
                        pub.publish(new_trajectory);
                }
            }
            sleep_timer.sleep();    
            
    }
    
    return 0;
}


