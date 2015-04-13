If you'd like to work from this steering algorithm, please make your own branch or package.

# steering_wcb38

A steering algorithm that converts lateral, heading and trip-dist errors into 
 a Twist.


### Speed Correction

speed correction = k*atan(j*trip_dist_err)
 k is the maximum correction factor
 j is the sensitivity correction (larger for faster changes)
 
 This gives a stable adjustment to speed based on how ahead or behind the robot
  is on its path. There is a known maximum (range of atan).

### Linear Velocity

Linear velocity is determined by adding the speed correction directly to the desired state velocity. If the robot is behind, the robot will accelerate ahead to catch up, and vice versa.

twist_cmd_.linear.x = des_state_vel_+speed_correction;


### Heading Error

The heading error was revised to account for lateral error. The equation for this is as follows:

heading_err = min_dang(des_state_phi+atan(2*lateral_err) - odom_phi)

 - min_dang = minimizes the angle, so that it will, for example, turn left from heading of 1 degree to heading of 359 degrees instead of turning 358 degrees to the right.
 - des_state_phi = published desired state heading
 - odom_phi = published current heading in odom
 - atan(2*laterall_err) = this is a correction that maps any lateral error to an adjustment of up to 90 degrees (directly facing/perpendicular to desired state x,y path). the *2 factor scales the adjustment so that it will be approximately 90 degrees at 1 meter of lateral error.

### Angular

Angular velocity is determined with proportional control based directly on the heading error calculation.

twist_cmd_.angular.z = heading_err*1

 

## Example usage

  rosrun steering_wcb38 simple_steering

## Running tests/demos
    
