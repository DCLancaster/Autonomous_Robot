#!/bin/bash

if [ -d "/home/dc/catkin_ws/src/407/Autonomous_Robot" ]
then
  cd /home/dc/catkin_ws/src/407/Autonomous_Robot || exit
  git add --all
  # shellcheck disable=SC2035
  git add *
  git commit -m "pushed by script"
  git push https://github.com/DCLancaster/Autonomous_Robot main
else
  echo false
fi