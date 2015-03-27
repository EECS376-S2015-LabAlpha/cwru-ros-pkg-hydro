# path_sender

This node is a combination of Jiawei's path planning node and Jean's desired state generator node. 
It takes in a bunch of Poses from the path sender, creates a bunch of PathSegments out of those poses
and then create desired states from those PathSegments. Those desired states are then passed to the 
steering node which publishes Twist commands based on the states.




## Example usage
To run the node, simple type:
'rosrun path_planner path_planner 
