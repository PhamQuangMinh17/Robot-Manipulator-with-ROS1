/* 
    Author: Pham Quang Minh
    email: quangminh2479@gmail.com
    Date: 14th July 2023

    robot_interface.h: C++ Header file for interfacing with robot hardware
*/
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <vector>
#pragma once
/* 
Class MinhRobotInterface is used for implement hardware interface for robot
*/
class MinhRobotInterface: public hardware_interface::RobotHW {
    public:
        MinhRobotInterface(ros::NodeHandle&);
        // Define usefule functions
        void update(const ros::TimerEvent& Event);
        void read();
        void write(ros::Duration TimeElapse);
    
    private:
        ros::NodeHandle NodeHandle_;
        // Instace is used to access the topics + services that re not proceeded by prefix
        ros::NodeHandle PrivateNodeHandle_;
        /* TimeElapse_: contain duration in sec since the last executed of control loop +
        the tine executaion of update function
        */
        ros::Duration TimeElapse_;
        /* UpdateFrequency: contain value of frequency for the execution of control loop
        */
        ros::Duration UpdateFrequency_;
        // Create Ros Timer: Execute constantly with certain frequency of control loop
        ros::Timer Looper_;
        // Instance for publisher class
        ros::Publisher HWPublisher_;
        // Instance for ServiceClient class
        ros::ServiceClient HWService_;

        hardware_interface::JointStateInterface JointStateInterface_;
        hardware_interface::PositionJointInterface JointPositionInterface_;
        // Pointer to class ControllerManager
        boost::shared_ptr<controller_manager::ControllerManager> ControllerManager_;
        
        // Vectors contain command sent to each motor (in decimal number)
        std::vector<double> Command_;
        std::vector<double> Position_;
        std::vector<double> Velocity_;
        std::vector<double> Effort_;
        // Name of each joint that controller will activate
        std::vector<std::string> Name_;
};