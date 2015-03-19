// program by William Baskin for lidar processing.
#include "lidar_space.h"

LidarSpace::LidarSpace(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
    ROS_INFO("Constructing LidarSpace");
    initializeSubscribers();
    initializePublishers();

    //test-wait for services/topics alive.
}

void LidarSpace::initializeSubscribers() {
    ROS_INFO("Initialize Subscribers");
    laser_sub_1 = nh_.subscribe("/base_laser1_scan", 1,
                                                &LidarSpace::LaserCallback, 
                                                this);
    laser_sub_2 = nh_.subscribe("/laser/scan", 1,
                                            &LidarSpace::LaserCallback, 
                                            this);
    laser_sub_2 = nh_.subscribe("/scan", 1,
                                            &LidarSpace::LaserCallback, 
                                            this);
    ROS_INFO("subscribers initialized.");
}

void LidarSpace::initializePublishers() {
    ROS_INFO("Initialize Publishers");
    lidar_out_ = nh_.advertise<lidar_space_detection::LidarSpaceMsg>("/lidar_spaces", 1, true);
    ROS_INFO("publishers initialized.");
}

void LidarSpace::LaserCallback(const sensor_msgs::LaserScan& scan) {
    ROS_INFO("LaserScan called");
    last_scan = scan;
    double angle_min = scan.angle_min;
    double angle_max = scan.angle_max;
    double angle_increment = scan.angle_increment;
    double range_min = scan.range_min;
    double range_max = scan.range_max;

    int start_index = 0;
    int min_space = 0;

    std::vector<lidar_space_detection::LidarSpaceSlice> new_slices;
    std::vector<geometry_msgs::Vector3> current_slice;
    lidar_space_detection::LidarSpaceSlice new_slice;

    //BigO(10n) runtime
    for(int i = 1; i <= (int) std::min((int) range_max, 10); i++) {
        current_slice.clear();
        start_index = 0;
        min_space = min_spacing((double) (i), angle_increment);
        //ROS_INFO("min space %d", min_space);

        for(int j = 0; j < (int) scan.ranges.size(); j++){
            //process each ping
            if(scan.ranges[j] < i || j == scan.ranges.size() -1 ) {
                if(start_index == 0) {
                    //ROS_INFO("start stop @ %f",scan.ranges[j]);
                }
                // if it is within the next slice
                if (j - start_index < min_space) {
                    start_index = j;
                }
                else {
                    // if the slice is big enough, push it into the list for the slice i
                    geometry_msgs::Vector3 option = convert_to_vec(start_index, j, angle_min, angle_increment, 
                                                                    (double) (i));
                    current_slice.push_back(option);
                    start_index = j;
                }
            }
        }
        // put the slice i list into the list of lists
        new_slice.spaces = current_slice;
        if((int) new_slice.spaces.size() > 1) {
            ROS_INFO("slice %d has size %d", i, (int) new_slice.spaces.size());
        }
        new_slices.push_back(new_slice);
    }
    // publish the lists for every level.
    last_publish.ranges = new_slices;
    last_publish.dist_increment = 1.0;
    lidar_out_.publish(last_publish);
}

int LidarSpace::min_spacing(double radius, double increment) {
    return (int) (1.5*((.3/radius)/increment));
}

geometry_msgs::Vector3 LidarSpace::convert_to_vec(int start_index, int end_index, double angle_min, double angle_increment, 
                                        double radius) {
    geometry_msgs::Vector3 to_return;
    to_return.x = radius;
    double start_angle = ((double)(start_index))*angle_increment+angle_min;
    double end_angle = ((double)(end_index))*angle_increment+angle_min;
    to_return.y = (start_angle+end_angle)/2;
    to_return.z = radius*(end_angle-start_angle);
    return to_return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_space_detection"); //name this node
    ros::NodeHandle nh;
    
    ROS_INFO("Main instantiating LidarSpace class");
    LidarSpace lidar_space(&nh);
    
    ROS_INFO("Spin!!");
    ros::spin();

    return 0;
}



