//
// Created by pocket on 24-2-11.
//

#include "humanoid_controllers/humanoidController.h"
#include "humanoid_dummy/gait/GaitKeyboardPublisher.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <thread>

using namespace ocs2;
using namespace humanoid;

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;

// init a GaitKeyboardPublisher ptr
std::shared_ptr<GaitKeyboardPublisher> gaitCommand;
double vx, vy, vyaw;
std::string desiredGait;

// gait command publisher thread
/*****
 * @brief 线程函数，用于发布gait command
 * @note 当速度从0变成非0时，发布一次trot，当从非0变成0时，发布一次stance
 * @note 仅在0和非0之间切换时发布
*/
void gaitCommandPublisher(){
    static double v_mod = 0;
    static double v_mod_last = 0;
    while(ros::ok()){
        v_mod = vx * vx + vy * vy + vyaw * vyaw;
        if(v_mod < 0.00001 && v_mod_last > 0.00001){
            gaitCommand->publishGaitCommandFromString("stance");
        }
        else if(v_mod > 0.00001 && v_mod_last < 0.00001)
        {
            gaitCommand->publishGaitCommandFromString(desiredGait);
        }
        v_mod_last = v_mod;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


/**
 * @brief cmd_vel callback
*/
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    vx = msg->linear.x;
    vy = msg->linear.y;
    vyaw = msg->angular.z;
}

/**
 * @brief desired_gait callback
*/
void desiredGaitCallback(const std_msgs::String::ConstPtr& msg){
    desiredGait = msg->data;
}


int main(int argc, char** argv){
    const std::string robotName = "humanoid";
    ros::Duration elapsedTime_;

    ros::init(argc, argv, "humanoid_joy_controller_node");
    ros::NodeHandle nh;

    // Get node parameters
    std::string gaitCommandFile;
    nh.getParam("/gaitCommandFile", gaitCommandFile);
    std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

    // init the gaitCommand ptr
    gaitCommand = std::make_shared<GaitKeyboardPublisher>(nh, gaitCommandFile, robotName, true);

    //---------------------------------------------------
    humanoid_controller::HybridJointInterface* robot_hw;
    //create a subscriber to pauseFlag
    humanoid_controller::humanoidController controller;
    if (!controller.init(robot_hw, nh)) {
        ROS_ERROR("Failed to initialize the humanoid controller!");
        return -1;
    }

    auto startTime = Clock::now();
    auto startTimeROS = ros::Time::now();
    controller.starting(startTimeROS);
    auto lastTime = startTime;

    //create a subscriber to cmd_vel
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmdVelCallback);

    //create a subscriber to String /desired_gait_str
    ros::Subscriber desired_gait_sub = nh.subscribe<std_msgs::String>("desired_gait_str", 1, desiredGaitCallback);

    //create a thread to spin the node
    std::thread spin_thread([](){
        ros::spin();
    });
    spin_thread.detach();

    // create a thread to publish gait command
    std::thread gait_command_publisher_thread(gaitCommandPublisher);
    gait_command_publisher_thread.detach();

    while(ros::ok()){
        const auto currentTime = Clock::now();
        // Compute desired duration rounded to clock decimation
        const Duration desiredDuration(1.0 / 500);

        // Get change in time
        Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime);
        elapsedTime_ = ros::Duration(time_span.count());
        lastTime = currentTime;

        // Control
        // let the controller compute the new command (via the controller manager)
        controller.update(ros::Time::now(), elapsedTime_);

        // Sleep
        const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
        std::this_thread::sleep_until(sleepTill);
    }

    return 0;
}