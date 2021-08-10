# fanuc_description

## Objective
>1. Derive DH parameters for the Fanuc robot.
>2. Translate DH parameters to URDF and visualize the Fanuc robot in RViz with urdf_tutorial display.launch

## Implementation
>The DH parameters were derived (as seen in doc/fanuc_dh.png) and then translated to an URDF file, that is used to represent the robot model.

![Image](https://raw.githubusercontent.com/rcoccaro4/roblab_ws/master/ex2_transforms/fanuc_description/doc/fanuc_dh.png) 

>Using RViz it is possible to visualize the descripted robot. In order to do so, the following commands are launched.
```
cd fanuc_description/robot
roslaunch urdf_tutorial display.launch model:=m20ia.urdf
``` 

![Image](https://raw.githubusercontent.com/rcoccaro4/roblab_ws/master/ex2_transforms/fanuc_description/doc/fanuc_rviz.png)
