#ifndef LABALPHA_LIDAR_SPACE_H_
#define LABALPHA_LIDAR_SPACE_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <lidar_space_detection/LidarSpaceMsg.h>

class LidarSpace
{
public:
	LidarSpace(ros::NodeHandle* nodehandle);

private:
	ros::NodeHandle nh_;

	ros::Subscriber laser_sub_;
	ros::Publisher lidar_out_;

	sensor_msgs::LaserScan last_scan;
	lidar_space_detection::LidarSpaceMsg last_publish;

	void initializeSubscribers();
	void initializePublishers();
    
    void LaserCallback(const sensor_msgs::LaserScan& scan);

    int min_spacing(double radius, double increment);

    geometry_msgs::Vector3 convert_to_vec(int start_index, int end_index, double angle_min, double angle_increment, double radius);

};

#endif //LABALPHA_LIDAR_SPACE_H_