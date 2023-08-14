/*
  Author: Pham Quang Minh
  email: quangminh2479@gmail.com
  Date: 14th July 2023
  Reference: Course: Robotics and ROS - Learn by Doing! Manipulators by Antonio Brandi 
*/
/* 
ROS use Radian for representing angle + angular position → If nodes send command in 
degree( for exp arduino board) → cannot be able to progress → Need Angle Converter Service 
*/
#include <ros/ros.h>
#include "minh_robot/AngleConvert_IF.h"
#include <math.h>

/*
    Function that is called every time the service Rad2Deg is called
    Input: Request message (angles in radians) and the Result message Type
    Function fills out the response message with the converted values 
    Output: Result message (angles in degrees)
*/
bool Convert_Rad2Deg(minh_robot::AngleConvert_IF::Request  &request,
         minh_robot::AngleConvert_IF::Response &response)
{
    response.base = static_cast<int>(((request.base + (M_PI/2))*180)/M_PI);
    response.shoulder = 180 - static_cast<int>(((request.shoulder + (M_PI/2))*180)/M_PI);
    response.elbow = static_cast<int>(((request.elbow + (M_PI/2))*180)/M_PI);
    response.gripper = static_cast<int>(((-request.gripper)*180)/(M_PI/2));
    return true;
}

/*
    Function that is called every time the service Rad2Deg is called
    Input: Request message (angles in degrees) and the Result message Type
    Function fills out the response message with the converted values 
    Output: Result message (angles in radianss)
*/
bool Convert_Deg2Rad(minh_robot::AngleConvert_IF::Request  &request,
         minh_robot::AngleConvert_IF::Response &response)
{
    response.base = ((M_PI*request.base) - ((M_PI/2)*180))/180;
    response.shoulder = (((180-request.shoulder)*M_PI)-((M_PI/2)*180))/180;
    response.elbow = ((M_PI*request.elbow) - ((M_PI/2)*180))/180;
    response.gripper = -((M_PI/2)*request.gripper)/180;
    return true;
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called angleconverting_service
    ros::init(argc, argv, "angleconverting_service");
    ros::NodeHandle NodeHandle;

    /* Inizialize two services for the angle conversions with function advertiseService*/
    // Receive input: name of service server and function to be called when request msg received

    ros::ServiceServer Rad2Deg = NodeHandle.advertiseService("Rad2Deg", Convert_Rad2Deg);
    ros::ServiceServer Deg2Rad = NodeHandle.advertiseService("Deg2Rad", Convert_Deg2Rad);
    
    // Print out message for the start of service
    ROS_INFO("Angle Converting Service Started");

    // Keep node run
    ros::spin();
    return 0;
}