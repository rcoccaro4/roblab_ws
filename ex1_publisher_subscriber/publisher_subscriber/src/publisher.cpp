#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_publisher =
        nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loopRate(10);
    
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0]="joint1";
    joint_state.position[0] = 1; 
    joint_state.name[1] ="joint2";
    joint_state.position[1] = 1;
    joint_state.name[2] ="joint3";
    joint_state.position[2] = 1;
    joint_state.name[3] ="joint4";
    joint_state.position[3] = 1;
    joint_state.name[4] ="joint5";
    joint_state.position[4] = 1;
    joint_state.name[5] ="joint6";
    joint_state.position[5] = 1;

    unsigned int count = 1;
    while (ros::ok()) {
        joint_state.header.stamp = ros::Time::now();
        
        for (int i = 0; i<6; i++) {
            joint_state.position[i]=(i+1)*count;
        }
        
        joint_publisher.publish(joint_state);
        ros::spinOnce();
        loopRate.sleep();
        count++;
    }
    ros::shutdown();
    return 0;
}