/*
  Author: Pham Quang Minh
  email: quangminh2479@gmail.com
  Date: 14th July 2023

  This script implements an Action Server that manages the execution
  of goals of the robot interfacing with moveit.
  Given a goal, it sends and execute a moveit trajectory

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <minh_robot/CommandIFAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>


class CommandServer
{
  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<minh_robot::CommandIFAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;

    // create messages that are used to publish result
    minh_robot::CommandIFResult result_;
    minh_robot::CommandIFFeedback persentage_;

    std::vector<double> arm_goal_;
    std::vector<double> gripper_goal_;

    moveit::planning_interface::MoveGroupInterface arm_move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_;

  public:
    // Constructor
    // function that inizialize the CommandIFAction class and creates 
    // a Simple Action Server from the library actionlib
    CommandServer(std::string name) :
      as_(nh_, name, boost::bind(&CommandServer::execute_cb, this, _1), false)
      , action_name_(name)
      , arm_move_group_("minh_robot_arm")
      , gripper_move_group_("minh_robot_gripper")
    {
      as_.start();
      ROS_INFO("Action Server started");
    }
    void execute_cb(const minh_robot::CommandIFGoalConstPtr& goal)
    {
      bool success = true;
      // start executing the action
      // based on the goal id received, send a different goal 
      // to the robot

      // Goal received
      for(int i=0;i<3;i++)
      {
        ROS_INFO("Arm Goal received: %f", goal->arm_pos[i]);
        if(i<2)
        {
          ROS_INFO("Gripper Goal received: %f", goal->gripper_pos[i]);
        }
      }
      // Set goal
      arm_goal_ = {goal->arm_pos[0], goal->arm_pos[1], goal->arm_pos[2]};
      gripper_goal_= {goal->gripper_pos[0], goal->gripper_pos[1]};
      
      // Sends a goal to the moveit API
      gripper_move_group_.setJointValueTarget(gripper_goal_);
      arm_move_group_.setJointValueTarget(arm_goal_);

      // Blocking functions below, will return after the execution
      gripper_move_group_.move();
      arm_move_group_.move();

      // Make sure that no residual movement remains
      arm_move_group_.stop();
      gripper_move_group_.stop();

      // Check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Action Server: %s - Status: Preempted", action_name_.c_str());
        as_.setPreempted();
        success = false;
      }
      // check if the goal request has been executed correctly
      if(success)
      {
        result_.success = true;
        ROS_INFO("Action Server: %s - Status: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
      }
    }
};


int main(int argc, char** argv)
{
  // Inizialize a ROS node called cmd_server
  ros::init(argc, argv, "cmd_server");
  CommandServer CommandServer("cmd_server");

  // keeps the node up and running
  ros::spin();
  return 0;
}
