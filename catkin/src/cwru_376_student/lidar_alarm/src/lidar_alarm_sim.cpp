// team program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message time


const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
int error_alert = 0;

//The width of our warn box, in meters
double l1 = 1.5;
//The length of our warn box, in meters
double l2 = 1;

std_msgs::Bool lidar_alarm_msg;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

bool ping_within_box_range(double angle_min, int angle_increment_,int i,double scan_distance, double width,double length){
    //tan(theta)=opp/adj
    //atan2(opp/adj) = theta... ok so now we have a postive theta. 
    double angle_from_normal = atan2(length,(width/2));

    //make theta neg and subtract the small portion below -90 degrees. We want this to the negative
    double left_critical_angle = (angle_min + 90)-angle_from_normal;

    //likewise for the right side, subtracting from the whole right part this time. This is positive
    double right_critical_angle = 90 - angle_from_normal;

    //angle from the -90 point
    double theta_normal = (angle_increment_*i)+(angle_min + 90);

    //raw ablge
    double raw_angle = angle_increment_*i + angle_min;

    //Check behind the lidar
    if((raw_angle < -90) || (raw_angle > 90)){
        double scan_distance_calc = ((width/2)/cos(std::abs((int)theta_normal % 180)));
        ROS_INFO("behind the lidar, max dist is: %f", scan_distance_calc);
        return scan_distance < ((width/2)/cos(std::abs((int)theta_normal % 180)));
    }
    //On the sides in the front
    else if((angle_increment_*i + angle_min < left_critical_angle) || (angle_increment_*i + angle_min > right_critical_angle)){
      double scan_distance_calc = (width/2)/sin(90 - theta_normal);
        ROS_INFO("sides in the front, max dist is: %f", scan_distance_calc);
        return scan_distance < ((width/2)/sin(90 - theta_normal));
    }
    //Directly in front
    else{
      double scan_distance_calc = (length/sin(theta_normal));
        ROS_INFO("Directly in front, max dist is: %f", scan_distance_calc);
        return scan_distance < (length/sin(theta_normal));
    }
}

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    error_alert = 0;
    //Initiate values
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
        //ping_index_ = (int) (0.0 -angle_min_)/angle_increment_;
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
    }
    //bounds for the lidar pings we want
    //double end_angle = angle_max_;
    //double start_angle = angle_min_;

    //Lets go from the start angle to the end angle and obtain all the pings in that range
    for(int i = (int)(angle_min_); i< (int) (angle_max_); i++){
    if (ping_within_box_range(angle_min_,angle_increment_,i,laser_scan.ranges[i],l1,l2)) {
      //if we find that at least two of them are within the danger zone the alert
        if(error_alert > 2){
            ROS_INFO("STOP ping dist in front = %f on ping %d",laser_scan.ranges[i], i);
            laser_alarm_=true;
      lidar_alarm_msg.data = true;
      ROS_INFO("lidar true");
      lidar_alarm_publisher_.publish(lidar_alarm_msg);
      return;
        }
        else error_alert++;
    }
    //lets reset our buffer if its not within range
    else {
        error_alert = 0;
        ROS_INFO("laser false");
        laser_alarm_=false;
        }
    }
 
   lidar_alarm_msg.data = laser_alarm_;
  ROS_INFO("lidar alarm %d", lidar_alarm_msg.data);
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg); 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh;
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}



