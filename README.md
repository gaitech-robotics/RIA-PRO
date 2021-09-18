# RIA-PRO


Installation

Note: Following steps are based on UBUNTU 18.04 with ROS-melodic

This step assumes that your PC is running with Ubuntu 18 and configured with ROS MELODIC and all required libraries 

Creating Work Space and cloning RIA PRO package

     mkdir –p ~/catkin_ws/src
     cd ~/catkin_ws/src
     git clone https://github.com/gaitech-robotics/RIA-PRO.git
     cd ~/catkin_ws && catkin_make

Install dependencies 

      sudo apt install ros-melodic-controller-manager ros-melodic-gazebo-ros-control ros-melodic-teleop-twist-keyboard ros-melodic-teleop-twist-joy ros-melodic-joy ros-melodic-slam-gmapping ros-melodic-move-base ros-melodic-map-server ros-melodic-amcl ros-melodic-teb-local-planner

