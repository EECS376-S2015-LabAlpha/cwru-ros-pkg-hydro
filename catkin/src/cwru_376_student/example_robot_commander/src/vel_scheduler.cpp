
// try this, e.g. with roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
// or:  roslaunch cwru_376_launchers stdr_glennan_2.launch 
// watch resulting velocity commands with: rqt_plot /robot0/cmd_vel/linear/x (or jinx/cmd_vel...)

//intent of this program: modulate the velocity command to comply with a speed limit, v_max,
// acceleration limits, +/-a_max, and come to a halt gracefully at the end of
// an intended line segment

// notes on quaternions:
/*
From:
http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/

qx = ax * sin(angle/2)
qy = ay * sin(angle/2)
qz = az * sin(angle/2)
qw = cos(angle/2)


so, quaternion in 2-D plane (x,y,theta):
ax=0, ay=0, az = 1.0

qx = 0;
qy = 0;
qz = sin(angle/2)
qw = cos(angle/2)

therefore, theta = 2*atan2(qz,qw)
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_datatypes.h>


// set some dynamic limits...
// const double v_max = 5.0; //1m/sec is a slow walk
const double v_min = 0.1; // if command velocity too low, robot won't move
const double a_max = 0.80; //1m/sec^2 is 0.1 g's
//const double a_max_decel = 0.1; // TEST
const double omega_max = 1.5; //1 rad/sec-> about 6 seconds to rotate 1 full rev
const double alpha_max = 0.5; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
const double DT = 0.050; // choose an update rate of 20Hz; go faster with actual hardware

// globals for communication w/ callbacks:
double odom_vel_ = 0.0; // measured/published system speed
double odom_omega_ = 0.0; // measured/published system yaw rate (spin)
double odom_x_ = 0.0;
double odom_y_ = 0.0;
double odom_yaw = 0.0;
double last_yaw = 0.0;

double dt_odom_ = 0.0;
ros::Time t_last_callback_;
double dt_callback_=0.0;

ros::Publisher vel_cmd_publisher;
geometry_msgs::Twist cmd_vel;
double last_rotation = 0.0;


// receive odom messages and strip off the components we want to use
// tested this OK w/ stdr

// receive the pose and velocity estimates from the simulator (or the physical robot)
// copy the relevant values to global variables, for use by "main"
// Note: stdr updates odom only at 10Hz; Jinx is 50Hz (?)

void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    //here's a trick to compute the delta-time between successive callbacks:
    dt_callback_ = (ros::Time::now() - t_last_callback_).toSec();
    t_last_callback_ = ros::Time::now(); // let's remember the current time, and use it next iteration

    if (dt_callback_ > 0.15) { // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
        dt_callback_ = 0.1; // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        ROS_WARN("large dt; dt = %lf", dt_callback_); // let's complain whenever this happens
    }
    
    // copy some of the components of the received message into global vars, for use by "main()"
    // we care about speed and spin, as well as position estimates x,y and heading
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    // see notes above for conversion for simple planar motion
    //double quat_z = odom_rcvd.pose.pose.orientation.z;
    //double quat_w = odom_rcvd.pose.pose.orientation.w;
    tf::Quaternion q(odom_rcvd.pose.pose.orientation.x, odom_rcvd.pose.pose.orientation.y, odom_rcvd.pose.pose.orientation.z, odom_rcvd.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odom_yaw = yaw;
    //odom_phi_ = 2.0*atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion

    // the output below could get annoying; may comment this out, but useful initially for debugging
    ROS_INFO("odom CB: x = %f, y= %f, yaw = %f, v = %f, omega = %f", odom_x_, odom_y_, odom_yaw, odom_vel_, odom_omega_);
}

//Used to get the difference between the last yaw and the current
double getDeltaYaw(double previous, double current){
    double delta;
    if((previous < -3)&&(current > 3)){
        delta = -((3.14159 + previous) + (3.14159 - current));
    }
    else if ((current < -3)&&(previous > 3)){
        delta = (3.14159 - previous) + (3.14159 + current);
    }
    else delta = current - previous;
    return delta;
}

//Used to see where we are in our trip
double getScheduledVelocity(double dist,double decel, double a, double v){
    
    if (dist<= 0.0) { // at goal, or overshot; stop!
            return 0.0;
    }
    else if (dist <= decel) { //possibly should be braking to a halt

        ROS_INFO("braking zone: v_sched = %f",v);
        return sqrt(2 * dist * a);
    }
    else { // not ready to decel, so target vel is v_max, either accel to it or hold it
        return v;
    }
}

//Check if we are starting up
double currentToScheduledChecker(double odomV,double scheduledV,double a){
    if (odomV < scheduledV) {  // maybe we halted, e.g. due to estop or obstacle;
            // may need to ramp up to v_max; do so within accel limits
            double v_test = odomV + a*dt_callback_; // if callbacks are slow, this could be abrupt
            // operator:  c = (a>b) ? a : b;
            return (v_test < scheduledV) ? v_test : scheduledV; //choose lesser of two options
            // this prevents overshooting scheduled_vel
    } else if (odomV > scheduledV) { //travelling too fast--this could be trouble
        // ramp down to the scheduled velocity.  However, scheduled velocity might already be ramping down at a_max.
        // need to catch up, so ramp down even faster than a_max.  Try 1.2*a_max.
        ROS_INFO("odom vel: %f; sched vel: %f",odomV,scheduledV); //debug/analysis output; can comment this out
        
        double v_test = odomV - 1.2 * a *dt_callback_; //moving too fast--try decelerating faster than nominal a_max

        return (v_test > scheduledV) ? v_test : scheduledV; // choose larger of two options...don't overshoot scheduled_vel
    } else {
        return scheduledV; //silly third case: this is already true, if here.  Issue the scheduled velocity
    }
}

//Used to make individual segments
void movesegment(double segment_length, ros::Rate rtimer, double v_max, int degrees){
    double segment_length_done = 0.0; // need to compute actual distance travelled within the current segment
    double start_x = 0.0; // fill these in with actual values once odom message is received
    double start_y = 0.0; // subsequent segment start coordinates should be specified relative to end of previous segment

    double scheduled_vel = 0.0; //desired vel, assuming all is per plan
    double new_cmd_vel = 0.1; // value of speed to be commanded; update each iteration
    double new_yaw_vel = 0.0;
    double scheduled_yaw_vel = 0.0;
    double dist_decel_rot = 0.5;
    double rotation_to_go = degrees * (3.14159 / 180);
    
    start_x = odom_x_;
    start_y = odom_y_;
    ROS_INFO("start pose: x %f, y= %f", start_x, start_y);

    // compute some properties of trapezoidal velocity profile plan:
    double T_accel = v_max / a_max; //...assumes start from rest
    double T_decel = v_max / a_max; //(for same decel as accel); assumes brake to full halt
    double dist_accel = 0.5 * a_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
    double dist_decel = 0.5 * a_max * (T_decel * T_decel);; //same as ramp-up distance
    double dist_const_v = segment_length - dist_accel - dist_decel; //if this is <0, never get to full spd
    double last_yaw = odom_yaw;

    while (ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted (or ctl-C)
    {
        ros::spinOnce(); // allow callbacks to populate fresh data
        ROS_INFO("last_yaw: %f odom_yaw: %f",last_yaw, odom_yaw);
        rotation_to_go = rotation_to_go - getDeltaYaw(last_yaw, odom_yaw);
        ROS_INFO("roation: %f",rotation_to_go);
        
        // compute distance travelled so far:
        double delta_x = odom_x_ - start_x;
        double delta_y = odom_y_ - start_y;
        segment_length_done = sqrt(delta_x * delta_x + delta_y * delta_y);
        ROS_INFO("dist travelled: %f", segment_length_done);
        double dist_to_go = segment_length - segment_length_done;

        scheduled_vel = getScheduledVelocity(dist_to_go,dist_decel,a_max,v_max);
        
        if(rotation_to_go<0){
            scheduled_yaw_vel = -getScheduledVelocity(std::abs(rotation_to_go),dist_decel_rot,alpha_max,omega_max);
        }
        else scheduled_yaw_vel = getScheduledVelocity(std::abs(rotation_to_go),dist_decel_rot,alpha_max,omega_max);
        
        
        new_cmd_vel = currentToScheduledChecker(odom_vel_,scheduled_vel,a_max);
        new_yaw_vel = currentToScheduledChecker(odom_omega_,scheduled_yaw_vel,alpha_max);
        
        cmd_vel.linear.x = new_cmd_vel;
        
        if(degrees == 0){
            cmd_vel.angular.z = 0;
            rotation_to_go = 0;
        }
        else {
            cmd_vel.angular.z = new_yaw_vel;// spin command; always zero, in this example
        }
        
        if (dist_to_go <= 0.0) { //uh-oh...went too far already!
            cmd_vel.linear.x = 0.0;  //command vel=0
        }
        
        //ROTATIONAL BIT
        if ((last_rotation > 0 && rotation_to_go < 0.0)||(last_rotation < 0 && rotation_to_go > 0.0)){ //uh-oh...went too far already!
            cmd_vel.angular.z = 0.0;  //command vel=0
        }
        
        vel_cmd_publisher.publish(cmd_vel); // publish the command to robot0/cmd_vel
        rtimer.sleep(); // sleep for remainder of timed iteration
        if (dist_to_go <= 0.0  &&  (((last_rotation > 0 && rotation_to_go < 0.0)||(last_rotation < 0 && rotation_to_go > 0.0))||(degrees==0)) )
            break; // halt this node when this segment is complete.
        // will want to generalize this to handle multiple segments
        // ideally, will want to receive segments dynamically as publications from a higher-level planner

        //Lets remember some things for the next iteration
        last_rotation = rotation_to_go;
        last_yaw = odom_yaw;
    }
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_scheduler"); // name of this node will be "minimal_publisher1"
    ros::NodeHandle nh; // get a ros nodehandle; standard yadda-yadda
    //create a publisher object that can talk to ROS and issue twist messages on named topic;
    // note: this is customized for stdr robot; would need to change the topic to talk to jinx, etc.
    vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("/robot0/odom", 1, odomCallback);
    ros::Rate rtimer(1 / DT); // frequency corresponding to chosen sample period DT; the main loop will run this fast

    // here is a crude description of one segment of a journey.  Will want to generalize this to handle multiple segments
    // define the desired path length of this segment

    cmd_vel.linear.x = 0.0; // initialize these values to zero
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // let's wait for odom callback to start getting good values...
    odom_omega_ = 1000000; // absurdly high
    ROS_INFO("waiting for valid odom callback...");
    t_last_callback_ = ros::Time::now(); // initialize reference for computed update rate of callback
    while (odom_omega_ > 1000) {
        rtimer.sleep();
        ros::spinOnce();
    }
    
    ROS_INFO("received odom message; proceeding");

    //Now we can take care of our segments by providing the distance, a timer, a spin(if needed) and an angle
    //Notice its not 90 degrees. Reason is in the update refresh rate being slow.
    movesegment(4.8, rtimer, 5 ,0);
    movesegment(0.0, rtimer, 0, -84);
    movesegment(12.25, rtimer, 5 ,0);
    movesegment(0.0, rtimer, 0, -87);
    movesegment(10.0, rtimer, 5 ,0);
    movesegment(0.0, rtimer, 0, -90);
    
    ROS_INFO("completed move distance");
}

