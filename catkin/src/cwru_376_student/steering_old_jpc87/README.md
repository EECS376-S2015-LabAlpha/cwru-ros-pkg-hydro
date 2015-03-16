# steering_wcb38

A steering algorithm that converts lateral, heading and trip-dist errors into 
 a Twist.

### LINEAR 
speed correction = -k*atan(j*trip_dist_err)
 k is the maximum correction factor
 j is the sensitivity correction (larger for faster changes)
 
 This gives a stable adjustment to speed based on how ahead or behind the robot
  is on its path. There is a known maximum (range of atan).
 This is directly added to the requested/desired velocity. Note, this may be
  forced to 0 if the modified heading error is too large.

### Angular
desired_heading = -k*atan(j*lateral_err)
 k is the correction magnitude (usually 1)
 j is the sensitivity correction (larger for faster changes)
 
 This gives a stable adjustment to the desired heading to correct for lateral
  error. The maximum angle is going to be 90 deg (1.57 radians) perpendicular to
  the path for large lateral error.
 The steering correction/modified heading error is the difference between this 
  and the heading error. This is converted to command_omega by the following:
 
controller_omega = steering_correction*2.0*UPDATE_RATE - odom_omega

 

## Example usage

 WIP

## Running tests/demos
    
