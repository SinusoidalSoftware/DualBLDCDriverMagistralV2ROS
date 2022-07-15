# DualBLDCDriverROS
Dual BLDC Driver (Magistral V2) Ros Hardware Interface Package
This package is created for building ros hardware interface of [Dual BLDC Driver Magistral V2] (https://sinusoidal.com.tr/magistralv2/).

# Contains
This package contains hardware interface source and header files to interface ros_control package.

Package contains 
- Communication Driver as comm_driver.cpp, 
- Checksum Calculation for Stm32 as crc32.cpp, 
- Hardware Interface as dual_driver.cpp and Driver node to be launched as sinusoidaldriver_node.cpp

# Connection
Dual BLDC Driver (Magistral V2)  Connection Diagram ![MagistralV2Connection](https://cloud.sinusoidal.com.tr/f/db977981733e418a8f1d/?dl=1)

# Requirements
Package created and tested with ROS Noetic.
Necessary ros packages,
* serial
* controller_manager
* geometry_msgs
* hardware_interface
* controller_interface
* roscpp
* rosparam_shortcuts
* std_msgs
* urdf
* joint_limits_interface
* diff-driver-controller
* move-base (optional)
* rqt-controlller-manager (optional)
* key-teleop (optional)


# Installation
* Clone the repository into catkin workspace's src folder.
* catkin_make
* source devel/setup.bash
* roslaunch sinusoidaldriver sinusoidaltest.launch

# Test 
* rosrun key_teleop key_teleop.py /key_vel:=/mobile_base_controller/cmd_vel
* rosrun rqt_gui rqt_gui

# Parameter Files
* Under src/sinusoidaldriver/config there are 2 .yaml files.
    - control.yaml file,
      -left_wheel -> joint name from urdf,
      -right_wheel -> joint name from urdf,
      -publish_rate -> must be same with driver setting,
      -wheel_seperation -> must be filled in meters
      -wheel_radius -> must be filled in meters
    - driver.yaml file,
      -left_wheel_name -> joint name from urdf,
      -right_wheel_name -> joint name from urdf,
      -baudrate -> 115200 (fixed),
      -device -> "/dev/ttyUSBX" X must be changed,
      -hall_counts_per_rev -> motor pole count * 6
      -driver_loop_rate -> must be same with driver setting,
      -left_wheel_reversed -> if motor reversed set true,
      -right_wheel_reversed -> if motor reversed reversed set true.
      
      



