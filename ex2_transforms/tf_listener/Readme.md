# tf_listener

## Objective
>Write one or more ROS nodes to compute and print the TF of the end-effector in all the reference frames of all joints. From the TF, compute the translational vector, the rotation matrix, the Euler angles and the axis-angle representation.

## Implementation
>The implementation consists of one node that serves as a listener on the topic /tf. The node then parses and prints on the stdout what he has listened to.