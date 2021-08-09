#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

void joint_states_callback(const sensor_msgs::JointState& msg) {
    for (int i = 0; i<6; i++) {
        ROS_INFO("I heard: that [%s] is at position [%f]", msg.name[i].c_str(), msg.position[i]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_states_listener");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = 
        nodeHandle.subscribe("joint_states",10,joint_states_callback);
    ros::spin();
    return 0;
}