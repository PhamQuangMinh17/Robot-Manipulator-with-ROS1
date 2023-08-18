/*
  Author: Pham Quang Minh
  email: quangminh2479@gmail.com
  Date: 14th July 2023
    Action client: Take input from user. Send target position to action server for execution.
*/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "minh_robot/CommandIFAction.h"
#include <iostream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_client");
    actionlib::SimpleActionClient<minh_robot::CommandIFAction> ActionServer("cmd_server");
    ROS_INFO("Waiting for action server");
    ActionServer.waitForServer();
    ROS_INFO("Action Server is ready.");
    minh_robot::CommandIFGoal goal;
    double ArmPosArray[3];
    double GripperPosArray[2];
    // Arm Position
    ROS_INFO("Enter target position respectively: x y z ");
    std::cin>> ArmPosArray[0] >> ArmPosArray[1] >> ArmPosArray[2];
    goal.arm_pos.clear();
    goal.arm_pos.push_back(ArmPosArray[0]);
    goal.arm_pos.push_back(ArmPosArray[1]);
    goal.arm_pos.push_back(ArmPosArray[2]);
    // Gripper Position
    int GripperSTT = 0;
    ROS_INFO("Enter gripper command: 0-gripper close 1-gripper open");
    std::cin>> GripperSTT;
    if(GripperSTT == 0)
    {
      // Gripper Close
        GripperPosArray[0] = 0;
        GripperPosArray[1] = 0;
    }
    else if(GripperSTT == 1)
    {
      // Gripper Open
        GripperPosArray[0] = -0.7;
        GripperPosArray[1] = 0.7;
    }
    else{
        // Do nothing
    }
    goal.gripper_pos.clear();
    goal.gripper_pos.push_back(GripperPosArray[0]);
    goal.gripper_pos.push_back(GripperPosArray[1]);
    ActionServer.sendGoal(goal);
    ActionServer.waitForResult(ros::Duration(60.0));
    auto result = ActionServer.getResult();
    ROS_INFO("Action Client: Action finished");
    return 0;
}
