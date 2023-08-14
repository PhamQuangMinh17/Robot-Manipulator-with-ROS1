/* 
  Author: Pham Quang Minh
  email: quangminh2479@gmail.com
  Date: 14th July 2023
*/

#include "minh_robot/robot_interface.h"
#include <std_msgs/UInt16MultiArray.h>
#include "minh_robot/AngleConvert_IF.h"
MinhRobotInterface::MinhRobotInterface(ros::NodeHandle& NodeHandle) : 
            NodeHandle_(NodeHandle), 
            PrivateNodeHandle_("~"),
            Position_(4, 0),
            Velocity_(4, 0),
            Effort_(4, 0),
            Command_(4, 0),
            Name_{"joint_1", "joint_2", "joint_3", "joint_4"}
{
    // Read from the parameter server server
    PrivateNodeHandle_.param("joint_names", Name_, Name_);
    /*
    Publisher: publish a message with array of integer to topic with buffer = 1000 (messages).
--> Arduino sub to this topic --> retrieve command for each motor 
    */
    HWPublisher_ = PrivateNodeHandle_.advertise<std_msgs::UInt16MultiArray>("/ArduinoBoard/RobotActuate", 1000);
    /*
     Use Interface to contact service server.--> contact service named /Rad2Deg
    */
    HWService_ = PrivateNodeHandle_.serviceClient<minh_robot::AngleConvert_IF>("/Rad2Deg");
    
    ROS_INFO("Starting Hardware Interface...");

    // Connect and register JointStateInterface
    /*
        Joint State Interface is used for interfacing with Hardware of robot to track status of robot
    joints including information: Name of joint, Current position, current velocity, current effort
    From hardware_interface library create JointStateHandle class. 
    */
    hardware_interface::JointStateHandle StateHandle_Joint_1(Name_.at(0), &Position_.at(0), &Velocity_.at(0), &Effort_.at(0));
    JointStateInterface_.registerHandle(StateHandle_Joint_1);
    hardware_interface::JointStateHandle StateHandle_Joint_2(Name_.at(1), &Position_.at(1), &Velocity_.at(1), &Effort_.at(1));
    JointStateInterface_.registerHandle(StateHandle_Joint_2);
    hardware_interface::JointStateHandle StateHandle_Joint_3(Name_.at(2), &Position_.at(2), &Velocity_.at(2), &Effort_.at(2));
    JointStateInterface_.registerHandle(StateHandle_Joint_3);
    hardware_interface::JointStateHandle StateHandle_Joint_4(Name_.at(3), &Position_.at(3), &Velocity_.at(3), &Effort_.at(3));
    JointStateInterface_.registerHandle(StateHandle_Joint_4);
    registerInterface(&JointStateInterface_);
    /* 
JointPositionInterface_ : track the current position and target command that sent to each joint
    */
    hardware_interface::JointHandle PositionHandle_Joint_1(JointStateInterface_.getHandle(Name_.at(0)), &Command_.at(0));
    JointPositionInterface_.registerHandle(PositionHandle_Joint_1);
    hardware_interface::JointHandle PositionHandle_Joint_2(JointStateInterface_.getHandle(Name_.at(1)), &Command_.at(1));
    JointPositionInterface_.registerHandle(PositionHandle_Joint_2);
    hardware_interface::JointHandle PositionHandle_Joint_3(JointStateInterface_.getHandle(Name_.at(2)), &Command_.at(2));
    JointPositionInterface_.registerHandle(PositionHandle_Joint_3);
    hardware_interface::JointHandle PositionHandle_Joint_4(JointStateInterface_.getHandle(Name_.at(3)), &Command_.at(3));
    JointPositionInterface_.registerHandle(PositionHandle_Joint_4);
    // Note: registerInterface() of RobotHW class is available in MinhRobotInterface class
    registerInterface(&JointPositionInterface_);

    ROS_INFO("Interfaces are registered.");


    ROS_INFO("Preparing the Controller Manager");

    ControllerManager_.reset(new controller_manager::ControllerManager(this, NodeHandle_));
    /*
     Initialize ROS Timer that trigger executation of function update()
    */
    UpdateFrequency_ = ros::Duration(0.1); // 0.1 second --> 10 hertz
    Looper_ = NodeHandle_.createTimer(UpdateFrequency_, &MinhRobotInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");
}

void MinhRobotInterface::update(const ros::TimerEvent& Event)
{
    ROS_INFO("Update Event");
    TimeElapse_ = ros::Duration(Event.current_real - Event.last_real);
    read();
    ControllerManager_->update(ros::Time::now(), TimeElapse_);
    write(TimeElapse_);
}

/*
    HW problems: Cannot receive the current position of motors when receive command
    --> no feedbacks for control loop --> Assume that command send and motor move the same
*/
void MinhRobotInterface::read()
{
    Position_.at(0) = Command_.at(0);
    Position_.at(1) = Command_.at(1);
    Position_.at(2) = Command_.at(2);
    Position_.at(3) = Command_.at(3);
}

void MinhRobotInterface::write(ros::Duration TimeElapse_)
{    
    // Create instance of message interface 
    minh_robot::AngleConvert_IF Service;
    // Request message for each joints with value from Command_ which are calculated from controller manager
    Service.request.base = Command_.at(0);
    Service.request.shoulder = Command_.at(1);
    Service.request.elbow = Command_.at(2);
    Service.request.gripper = Command_.at(3);

    // Call service server and show the response of the service
    if (HWService_.call(Service)) 
    {
        // If Server can be called and Received the response --> compose the array for response message
        std::vector<unsigned int> AngleInDeg;
        AngleInDeg.push_back(Service.response.base);
        AngleInDeg.push_back(Service.response.shoulder);
        AngleInDeg.push_back(Service.response.elbow);
        AngleInDeg.push_back(Service.response.gripper);
        std_msgs::UInt16MultiArray Message;
        Message.layout.dim.push_back(std_msgs::MultiArrayDimension());
        Message.layout.dim[0].size = AngleInDeg.size();
        Message.layout.dim[0].stride = 1;
        Message.data.clear();
        Message.data.insert(Message.data.end(), AngleInDeg.begin(), AngleInDeg.end());
        HWPublisher_.publish(Message);
    }
    else
    {
        ROS_ERROR("Failed to call service Rad2Deg");
    }
}

/* Main function */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "MinhRobot_Interface_Node");
    ros::NodeHandle NodeHandle;
    ros::MultiThreadedSpinner Spinner(2);
    MinhRobotInterface robot(NodeHandle);
    Spinner.spin();
    return 0;
}