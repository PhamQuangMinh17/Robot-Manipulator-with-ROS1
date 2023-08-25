#     Robot-Manipulator-with-ROS1

## Video demostration:
- Simulation:

Part 1: https://www.youtube.com/watch?v=n7D3fjecdEo

Part 2: https://www.youtube.com/watch?v=IPjg3dSxxC4

- Real-life 3D-printed Robot:


## Project Description

Control 3D-printed robot arm with ROS1 and Arduino Uno. 


## Prerequisite
### Hardware
- Arduino Uno
- 4 Servo Motors SG90
- Jumper wires
- Botl and Nuts
- 3D printed robot parts from **https://www.thingiverse.com/thing:1015238**

### Software
- **Linux:**
This project was built on **ubuntu 18.04**. You can try with other version of ubuntu but please keep in mind the supported ROS Version. 
- **ROS1**:
Install Any version of ROS 1 ( Recommend **ROS Melodic** since this project was built with ROS Melodic)
- **Programming Language:**
C++, Python, XML --> Visual studio is recommended for better managing files of project.
- **Arduino IDE**
Install Arduino IDE version 1.8.19 for Linux Debian.
- **ROS Packages and tools**:
gazebo-ros-control, rosserial, rosserial-arduino, roboticsgroup-upatras-gazebo-plugins, actionlib, actionlib-tools (for testing), joint-state-publisher-gui, moveit.
- **Other Notes**
Please check whether gazebo and Rviz are working normally on your ubuntu. Re-install them if they are not working properly.
(Optional) Visual Studio: Installing C++, Python and CMake Tools, XML  in Visual Studio

## Installation
- Create your new ROS Workspace at /usr/home/"name" ("name" is arbitrary).
- Open new ROS workspace in terminal. Clone this repository to your newly created ROS workspace.
- Build ROS Workspace:

`$ catkin_make`

- Source the project:

`$ source devel/setup.bash`

You can do it permanently by:

`$ gedit ~/.bashrc`

Copy and paste the setup path of your project to bashrc file.For example:

`source ~/arduinobot_ws/devel/setup.bash`


## Usage

### Control robot in simulation version

- On Terminal 1, Launch the simulation version:

`$ roslaunch minh_robot simulation_final.launch`

- On Terminal 2, start the command server:

`$ rosrun minh_robot cmd_server
`
- On Terminal 3, start the command client:

`$ rosrun minh_robot cmd_client`

- Input position x,y,z that you want robot to move to inside client node.
- Input status of gripper (Open/close = 0/1) after input position.

### Launch the 3D-printed robot system:

To be updated

## Debugging

Use rostopic list to see available ROS topics.

Use rosservice list to see available ROS service.

Use rosparam list to see available ROS parameters on Parameter server.

## Acknowledgement
- 3D CAD source: Antonio Brandy and EEZYbotARM.
- Initial idea: course **Robotics and ROS Learn byDoing Manipulators** 

## Other

To be updated soon 

## Contact me

Email: quangminh2479@gmail.com

LinkedIn: https://www.linkedin.com/in/quang-minh-pham-025612221/
