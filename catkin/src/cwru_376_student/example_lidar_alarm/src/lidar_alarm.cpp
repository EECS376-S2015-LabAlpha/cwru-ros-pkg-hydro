// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message time
#include <std_msgs/String.h>


const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_="STOP";
int error_alert = 0;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
    ROS_INFO("angle min is: %f", angle_min_);
    ROS_INFO("angle max is: %f", angle_max_);
        angle_increment_ = laser_scan.angle_increment;
    ROS_INFO("angle increment is: %f", angle_increment_);
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
    ROS_INFO("range min is: %f, range max is: %f", range_min_, range_max_);
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
    }
    //Set the angle of the Lidar sweep here, from (negative) start_angle, to (positive) end_angle
    double end_angle = 1.047197551;
    double start_angle = -1.047197551;
    //-1.047197551 is equivalent to 60 degrees, so look at the range your supposed to obtain
    for(int i = (int)((start_angle - angle_min_)/angle_increment_); i< (int) ((end_angle - angle_min_)/angle_increment_); i++){
      if (laser_scan.ranges[i] < 1) {
          //Throw the error alert after 3 adjacent Lidar points that are all too close
          if(error_alert > 2){
              ROS_INFO("STOP ping dist in front = %f on ping %d",laser_scan.ranges[i], i);
              laser_alarm_="STOP";
          }
          else error_alert++;
      }
      else {
        if(error_alert != 0)
            error_alert--;
        laser_alarm_="PROCEED";
      }
    }
   /*ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   if (ping_dist_in_front_<MIN_SAFE_DISTANCE) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
   }
   else {
       laser_alarm_=false;
   }*/
   std_msgs::String lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh;
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::String>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}


