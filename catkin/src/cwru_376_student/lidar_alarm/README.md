# lidar_alarm

The lidar pings the area in front of it and records the distance from the lidar to that point (distance is set to infinite if maximum range is reached).
If the pings indicate that an object is close enough, then the lidar alarm will publish a STOP message. By default, the lidar sweeps from -120 to +120 degrees (with 0 
being the point directly in front); we can adjust this angle to sweep narrower (-60 to +60 degrees). 

## Example usage

By subscribing to "lidar_alarm" one can listen for a bool representing wheather the safe zone has been breached as follows:

ros::Subscriber lidar_sub = n.subscribe("/lidar_alarm",1,callback);

where std_msgs::Bool& msg would be input parameter as a bool

## Running tests/demos
    
