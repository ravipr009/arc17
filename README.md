# Amazon Robotics Challenge 2017 Stowing Operation by IITK-TCS
This repository is a is a collection of many essential robotics package needed to build a autonomous robot for warehouse automation.
Packages:

    iitktcs_calibration: Package for estimation and testing calibration of 3D sensor with the robot manipulator.
    iitktcs_controller: System architecture central process which is responsible for control of operations for the manipulator.
    iitktcs_motion_planner: Package for motion planning algorithms.
    iitktcs_msgs_srvs: Package for building all ROS messages and service files.
    iitktcs_pose_estimation: Package for 3D model fitting and pose estimation algorithms.
    iitktcs_robot_description: Package for xacro urdf robot description.
    iitktcs_startup_kit: Package for handling operations during training of new set of objects.
    iitktcs_ur10_ensenso_g2_suction_moveit_config: Moveit package.
    iitktcs_utils: Package defining additional set of utility nodes.
    json_maker: Package for writing and reading json files.
    universal_robot: Package from universal robot.
    ur_modern_driver: Control package for communication with the universal robot.


## How to install 
1. Be sure to have ros and moveit installed 
2. create a new catkin workspace catkin_ws:
3. Creat a src folder catkin_ws/src
3. clone this folder in the src folder
**git@github.com:ravipr009/arc17.git**
4. **catkin_make **
5. Source ros in the repository 
**source devel/setup.sh**

**NB: remember to source in every new terminal that you open**


## Ensure that the robot eternet cable and the computer cable are connected correctly to the router switch

## INFO on how to Set the computer on the right IP address:
How to find the right IP address? 
The IP address is composed by 4 number A.B.C.D. The first three need to be set as the same of the one set in the robot when it was installed. Please ask for this number. This number is stored in the NETWORK sction of the Desk interface of the robot. 
 The number D need to be different than the robot one because the robot and the computer do need to be in different IPs. 
Computer IP is for example A.B.C.E. For example, the robot is 172.16.0.2 and the computer network on 172.16.0.1. The netmask is 255.255.255.0 thas is also specified in the Network session of the DESK interface. 
**<computer_ip>=172.16.0.1
<robot_ip>=172.16.0.2**
## Add the ROS_IP information in the bash of your terminal
1. Open the bash file
gedit ~/.bashrc
2. Paste this two lines
export ROS_IP=<computer_ip>
export ROS_MASTER_URI=http://<computer_ip>:11311

