# lidar_alarm

The lidar pings the area in front of it and records the distance from the lidar to that point (distance is set to infinite if maximum range is reached).
If the pings indicate that an object is close enough, then the lidar alarm will publish a STOP message (seen as a true through the publisher). Else it will remain a false.
A box of width l1 and length l2 is set up so that any pings within this box will result in a STOP message. The average of 3 is taken to ensure accuracy. The width is taken to be half on one side of the robot and the other half on the other. The length is to the front of the lidar while the the back goes as far as the lidar permits.  

## Example usage

By subscribing to "lidar_alarm" one can listen for a bool representing wheather the safe zone has been breached as follows:

ros::Subscriber lidar_sub = n.subscribe("/lidar_alarm",1,callback);

where std_msgs::Bool& msg would be input parameter as a bool

## Running tests/demos
    
