//
// Created by pocket on 24-2-11.
//

#include "humanoid_controllers/humanoidController.h"

bool pause_flag = false;

void pauseCallback(const std_msgs::Bool::ConstPtr& msg){
    pause_flag = msg->data;
    std::cerr << "pause_flag: " << pause_flag << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "humanoid_controller_node");
    ros::NodeHandle nh;
    //create a subscriber to pauseFlag
    ros::Subscriber pause_sub = nh.subscribe<std_msgs::Bool>("pauseFlag", 1, pauseCallback);
    ocs2::humanoid::humanoidCheaterController controller;
    if (!controller.init(nh)) {
        ROS_ERROR("Failed to initialize the humanoid controller!");
        return -1;
    }

    auto startTime = ros::Time::now();
    controller.starting(startTime);

    while(ros::ok()){
        if (!pause_flag)
        {
            auto currentTime = ros::Time::now();
            controller.update(currentTime, ros::Duration(0.002));
        }
        ros::spinOnce();
    }

    return 0;
}