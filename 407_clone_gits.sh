#!/bin/sh

cd ~/catkin_ws/src/407/ || exit
if [ -d "/home/dc/catkin_ws/src/407/Autonomous_Robot" ]
then
  echo Autonomous_Robot already exists
else
  git clone https://github.com/DCLancaster/Autonomous_Robot
fi

if [ -d "/home/dc/catkin_ws/src/407/navigation" ]
then
  echo navigation already exists
else
  git clone -b melodic-devel https://github.com/ros-planning/navigation
fi

if [ -d "/home/dc/catkin_ws/src/407/slam_gmapping" ]
then
  echo slam_gmapping already exists
else
  git clone https://github.com/DCLancaster/Autonomous_Robot
fi

if [ -d "/home/dc/catkin_ws/src/407/m_explore" ]
then
  echo m_explore already exists
else
  git clone https://github.com/DCLancaster/Autonomous_Robot
fi
